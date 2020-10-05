#include <iostream>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/frustum_culling.h>

#include <graph_slam/depthmap_to_pointcloud_converter.h>

namespace graph_slam
{

DepthmapToPointCloudConverter::DepthmapToPointCloudConverter(const Eigen::Matrix3f& intrinsics) :
      intrinsics_(intrinsics)
{
}

DepthmapToPointCloudConverter::~DepthmapToPointCloudConverter() = default;

Eigen::Matrix3f DepthmapToPointCloudConverter::get_intrinsics() const { return intrinsics_; }

void DepthmapToPointCloudConverter::set_intrinsics(const Eigen::Matrix3f& new_intrinsic)
{
    intrinsics_ = new_intrinsic;
}

Eigen::Matrix3Xf
DepthmapToPointCloudConverter::inverse_project_depthmap_into_3d(const cv::Mat& depthmap)
{
    // depthmap's size of column (width), row (height), and total number of pixels
    int depthmap_col = depthmap.cols;
    int depthmap_row = depthmap.rows;
    int total_pixels = depthmap_col * depthmap_row;

    // creating column index vector : resulting in a row vector (1,total_pixels)
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> col_idx_flat_row_vec =
        Eigen::RowVectorXf::LinSpaced(depthmap_col, 0, depthmap_col - 1).replicate(depthmap_row, 1);
    col_idx_flat_row_vec.resize(1, total_pixels);

    // creating row index vector : resulting in a row vector (1,total_pixels)
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_idx_flat_row_vec =
        Eigen::VectorXf::LinSpaced(depthmap_row, 0, depthmap_row - 1).replicate(1, depthmap_col);
    row_idx_flat_row_vec.resize(1, total_pixels);

    // creating row matrix filled with ones : resulting in a row vector (1,total_pixels)
    auto one_flat_row_vec = Eigen::MatrixXf::Ones(1, total_pixels);

    // getting depth value inside a 2D depth map as a row vector (1,total_pixels)
    Eigen::MatrixXf depth_flat_row_vec = convert_depthmap_to_eigen_row_matrix(depthmap);

    Eigen::Matrix3Xf points(3, total_pixels);
    points.row(0) = col_idx_flat_row_vec;
    points.row(1) = row_idx_flat_row_vec;
    points.row(2) = one_flat_row_vec;

    return intrinsics_.inverse() * points * depth_flat_row_vec.asDiagonal();
}

void DepthmapToPointCloudConverter::get_pcl_pointcloud(
    const cv::Mat& depthmap, int subsample_factor, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_pc)
{
    auto points_3d = inverse_project_depthmap_into_3d(depthmap);

    int valid_points_cnt = 0;
    int total_points = points_3d.cols();

    // reserve memory space for optimization
    output_pc->reserve(total_points);

    for (int i = 0; i < total_points; i += subsample_factor)
    {
        if (points_3d(2, i) >= depth_lower_limit && points_3d(2, i) < depth_upper_limit)
        {
            output_pc->push_back(pcl::PointXYZ(points_3d(0, i), points_3d(1, i), points_3d(2, i)));
            valid_points_cnt++;
        }
    }

    output_pc->width = valid_points_cnt;
    output_pc->height = 1;
    output_pc->is_dense = true;
    output_pc->resize(valid_points_cnt);
}

void DepthmapToPointCloudConverter::save_pointcloud_to_pcd(
    const cv::Mat& depthmap, int subsample_factor, const std::string& save_file_path)
{
    if (save_file_path.empty())
    {
        std::cout << "Please specify the stored path. Job Aborted." << std::endl;
        return;
    }

    std::string complete_store_path = save_file_path + "saved_pcd.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    get_pcl_pointcloud(depthmap, subsample_factor, cloud_to_save);
    pcl::io::savePCDFileASCII(complete_store_path, *cloud_to_save);
}

void DepthmapToPointCloudConverter::apply_statistical_outlier_removal_filtering(
    const int mean_k_value, const float std_dev_multiplier,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(pointcloud);
    sor.setMeanK(mean_k_value);
    sor.setStddevMulThresh(std_dev_multiplier);
    sor.filter(*pointcloud);
}

void DepthmapToPointCloudConverter::apply_approximate_voxel_grid_filtering(
    const std::vector<float>& leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> avg;
    avg.setInputCloud(pointcloud);
    avg.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    avg.filter(*pointcloud);
}

void DepthmapToPointCloudConverter::apply_voxel_grid_filtering(
    const std::vector<float>& leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(pointcloud);
    vg.setLeafSize(leaf_size[0], leaf_size[1], leaf_size[2]);
    vg.filter(*pointcloud);
}

void DepthmapToPointCloudConverter::apply_frustum_culling(
    float v_fov, float h_fov, float near_plane_dist, float far_plane_dist,
    const Eigen::Matrix4f& cam_pose, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    pcl::FrustumCulling<pcl::PointXYZ> fc;
    fc.setInputCloud(pointcloud);
    fc.setVerticalFOV(v_fov);
    fc.setHorizontalFOV(h_fov);
    fc.setNearPlaneDistance(near_plane_dist);
    fc.setFarPlaneDistance(far_plane_dist);
    fc.setCameraPose(cam_pose);
    fc.filter(*pointcloud);
}

Eigen::MatrixXf
DepthmapToPointCloudConverter::convert_depthmap_to_eigen_row_matrix(const cv::Mat& depthmap)
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mtrx;
    cv::cv2eigen(depthmap, mtrx);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_mtrx = mtrx;

    row_mtrx.resize(1, depthmap.rows * depthmap.cols);

    // mm to m conversion
    row_mtrx /= 1000.0;

    return row_mtrx;
}

}  // namespace graph_slam
