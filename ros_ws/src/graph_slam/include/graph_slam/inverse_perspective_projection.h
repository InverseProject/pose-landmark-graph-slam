#pragma once

#include <string>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>

namespace graph_slam{
/*
 * Intrinsic Matrix (K)
 * |f_u   0   o_u|
 * | 0   f_v  0_v|
 * | 0    0    1 |
 * 
 * Inverse of Intrinsic Matrix (K^-1)
 * |1/f_u    0    -o_u/f_u|
 * |  0    1/f_v  -o_v/f_v|
 * |  0      0        1   |
 * 
 * f_u -> focal length (horizontal)
 * f_v -> focal length (vertical)
 * o_u -> principal point (horizontal)
 * o_v -> principal point (vertical)
 */

class InverseProjection
{
    public:
        InverseProjection(const Eigen::Matrix3f intrinsic, const cv::Mat depthmap);
        ~InverseProjection();
        Eigen::Matrix3f get_intrinsic() const;
        void set_intrinsic(const Eigen::Matrix3f new_intrinsic);
        cv::Mat get_depthmap() const;
        void set_depthmap(const cv::Mat new_depthmap);
        Eigen::Matrix3f get_intrinsic_inverse(const Eigen::Matrix3f& m); 
        Eigen::MatrixXf convert_cv_mat_to_flat_eigen_mat();
        Eigen::Matrix3Xf get_pointcloud_from_depthmap();
        pcl::PointCloud<pcl::PointXYZ> get_pcl_pointcloud();
        void save_pointcloud_to_pcd(std::string save_file_path);


    private:
        Eigen::Matrix3f intrinsic_matrix_;
        Eigen::Matrix3f intrinsic_matrix_inverse_;
        cv::Mat depthmap_;
        int depthmap_row_, depthmap_col_, total_pixels_;
};
}