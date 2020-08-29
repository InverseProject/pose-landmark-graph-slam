
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <graph_slam/inverse_perspective_projection.h>

namespace graph_slam{
InverseProjection::InverseProjection(const Eigen::Matrix3f intrinsic, const cv::Mat depthmap)
{
    intrinsic_matrix_ = intrinsic;
    // intrinsic_matrix_inverse_ = get_intrinsic_inverse(intrinsic_matrix_);
    intrinsic_matrix_inverse_ = intrinsic_matrix_.inverse();
    depthmap_ = depthmap;
    depthmap_.convertTo(depthmap_, CV_32F);
    depthmap_row_ = depthmap_.rows;
    depthmap_col_ = depthmap_.cols;
    total_pixels_ = depthmap_row_ * depthmap_col_;
}

InverseProjection::~InverseProjection() = default;

Eigen::Matrix3f InverseProjection::get_intrinsic() const
{
    return intrinsic_matrix_;
}

void InverseProjection::set_intrinsic(const Eigen::Matrix3f new_intrinsic)
{
     intrinsic_matrix_ = new_intrinsic;
     intrinsic_matrix_inverse_ = get_intrinsic_inverse(intrinsic_matrix_);
}

cv::Mat InverseProjection::get_depthmap() const
{
    return depthmap_;
}

void InverseProjection::set_depthmap(const cv::Mat new_depthmap)
{
    depthmap_ = new_depthmap;
    depthmap_.convertTo(depthmap_, CV_32F);
    depthmap_row_ = depthmap_.rows;
    depthmap_col_ = depthmap_.cols;
    total_pixels_ = depthmap_row_ * depthmap_col_;
}

Eigen::Matrix3f InverseProjection::get_intrinsic_inverse(const Eigen::Matrix3f& m)
{
    Eigen::Matrix3f inv_K;
    inv_K << 1/m(0,0), 0.0     , -m(0,2)/m(0,0),
             0.0     , 1/m(1,1), -m(1,2)/m(1,1),
             0.0     , 0.0     , 1.0;

    return inv_K;
}

Eigen::MatrixXf InverseProjection::convert_cv_mat_to_flat_eigen_mat()
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> mtrx;
    cv::cv2eigen(depthmap_, mtrx);
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> row_mtrx = mtrx;
    row_mtrx.resize(1, total_pixels_);
    row_mtrx /= 1000; // mm to m conversion
    return row_mtrx;
}

Eigen::Matrix3Xf InverseProjection::get_pointcloud_from_depthmap()
{
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> u_flat_mtrx = Eigen::RowVectorXf::LinSpaced(depthmap_col_, 0, depthmap_col_-1).replicate(depthmap_row_, 1);
    u_flat_mtrx.resize(1, total_pixels_);

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> v_flat_mtrx = Eigen::VectorXf::LinSpaced(depthmap_row_, 0, depthmap_row_-1).replicate(1, depthmap_col_);
    v_flat_mtrx.resize(1, total_pixels_);

    auto one_flat_mtrx = Eigen::MatrixXf::Ones(1, total_pixels_);

    Eigen::MatrixXf z_vector = convert_cv_mat_to_flat_eigen_mat();

    Eigen::Matrix3Xf points(3, total_pixels_);
    points.row(0) = u_flat_mtrx;
    points.row(1) = v_flat_mtrx;
    points.row(2) = one_flat_mtrx;

    return intrinsic_matrix_inverse_ * points * z_vector.asDiagonal();
}

pcl::PointCloud<pcl::PointXYZ> InverseProjection::get_pcl_pointcloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    auto points_3d = get_pointcloud_from_depthmap();

    int valid_points_cnt = 0;
    for(int i=0; i<points_3d.cols(); i++)
    {
        if(points_3d(2,i) < 65.535){
            cloud.push_back(pcl::PointXYZ(points_3d(0,i), points_3d(1,i), points_3d(2,i)));
            valid_points_cnt++;
        }
    }

    cloud.width = valid_points_cnt;
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    return cloud;
}

void InverseProjection::save_pointcloud_to_pcd(std::string save_file_path)
{
    std::string file_name = "saved_pcd.pcd";
    if(!save_file_path.empty()) save_file_path += file_name;
    
    auto cloud_to_save = get_pcl_pointcloud();
    pcl::io::savePCDFileASCII(save_file_path, cloud_to_save);
}
}