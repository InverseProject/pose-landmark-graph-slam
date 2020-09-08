#include <iostream>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

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

    // TODO(Sean) : optimize this line instead of multiplying (N,N) diagonal matrix
    return intrinsics_.inverse() * points * depth_flat_row_vec.asDiagonal();
}

void DepthmapToPointCloudConverter::get_pcl_pointcloud(
    const cv::Mat& depthmap, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_pc,
    int subsample_factor = 1)
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
    const cv::Mat& depthmap, const std::string& save_file_path, int subsample_factor = 1)
{
    if (save_file_path.empty())
    {
        std::cout << "Please specify the stored path. Job Aborted." << std::endl;
        return;
    }

    std::string complete_store_path = save_file_path + "saved_pcd.pcd";

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_to_save =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    get_pcl_pointcloud(depthmap, cloud_to_save, subsample_factor);
    pcl::io::savePCDFileASCII(complete_store_path, *cloud_to_save);
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
