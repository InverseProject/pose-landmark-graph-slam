#pragma once

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/impl/point_types.hpp>
#include <string>

namespace graph_slam
{

/**
 * Library class for converting 2D depthmap into 3D point clouds.
 *
 */

class DepthmapToPointCloudConverter
{

public:
    /**
     * Constructor
     *
     * @param intrinsics (const Eigen::Matrix3f&) : 3x3 camera intrinsics
     *
     * camera intrinsics (K)
     * |f_u   0   o_u|
     * | 0   f_v  0_v|
     * | 0    0    1 |
     *
     * (f_u, f_v) -> focal length (horizontal, vertical)
     * (o_u, o_v) -> principal point (horizontal, vertical)
     */
    DepthmapToPointCloudConverter(const Eigen::Matrix3f& intrinsics);

    /**
     * Destructor
     */
    ~DepthmapToPointCloudConverter();

    /**
     * Getter function to obtain stored intrinsics
     *
     * @return (Eigen::Matrix3f) : 3x3 camera intrinsics
     */
    Eigen::Matrix3f get_intrinsics() const;

    /**
     * Setter function to replace stored camera intrinsics
     *
     * @param new_intrinsics (const Eigen::Matrix3f&) : new 3x3 camera intrinsics
     */
    void set_intrinsics(const Eigen::Matrix3f& new_intrinsics);

    /**
     * This function takes 2D depth map and inverse project onto 3D
     * We can project every pixel in a 2D depth map to a 3D point
     * with the following matrix multiplications
     * [X,Y,Z] = K^-1 * [u,v,1] * z
     *
     * @param depthmap (const cv::Mat&) : 2D depth map with a size of (height, width)
     * @return (Eigen::Matrix3Xf) : (3,N) matrix where it contains N nubmer of points of (x,y,z)
     */
    Eigen::Matrix3Xf inverse_project_depthmap_into_3d(const cv::Mat& depthmap);

    /**
     * This function takes 2D depth map and stores a pcl point cloud in output_pc
     *
     * @param depthmap (const cv::Mat&) : 2D depth map with a size of (height, width)
     * @param output_pc (pcl::PointCloud<pcl::PointXYZ>::Ptr&) : converted point cloud as a
     * @param subsample_factor (int) : subsample factor enables sub sampling entire point cloud
     * reference
     */
    void get_pcl_pointcloud(
        const cv::Mat& depthmap, pcl::PointCloud<pcl::PointXYZ>::Ptr& output_pc,
        int subsample_factor);

    /**
     * This function saves converted point clouds into pcd format
     *
     * @param depthmap (const cv::Mat&) : 2D depth map with a size of (height, width)
     * @param save_file_path (std::string) : path to store point clouds as pcd. If
     * @param subsample_factor (int) : subsample factor enables sub sampling entire point cloud
     */
    void save_pointcloud_to_pcd(
        const cv::Mat& depthmap, const std::string& save_file_path, int subsample_factor);

    /**
     * It applies statistical outlier removal filtering on incoming point cloud.
     * Reference:
     * https://pointclouds.org/documentation/classpcl_1_1_statistical_outlier_removal.html
     *
     * @param mean_k_value (const int): number of nearest neighbors
     * @param std_dev_multiplier (const float): standard deviation multiplier for the distance
     * threshold calculation. Distance threshold = mean + std_dev_multiplier * std_dev.
     * @param pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr&): incoming PCL point cloud pointer.
     * After appyling the filter, the caller already has filtered pointcloud at the same pointer.
     */
    static void apply_statistical_outlier_removal_filtering(
        const int mean_k_value, const float std_dev_multiplier,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

    /**
     * It applies approximate voxel grid filtering on incoming point cloud.
     * Reference: https://pointclouds.org/documentation/classpcl_1_1_approximate_voxel_grid.html
     *
     * @param leaf_size (const std::vector<float>&): voxel dimension [x,y,z]
     * @param pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr&): incoming PCL point cloud pointer.
     * After appyling the filter, the caller already has filtered pointcloud at the same pointer.
     */
    static void apply_approximate_voxel_grid_filtering(
        const std::vector<float>& leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

    /**
     * It applies voxel grid filtering on incoming point cloud.
     * Reference: https://pointclouds.org/documentation/classpcl_1_1_voxel_grid.html
     *
     * @param leaf_size (const std::vector<float>&): voxel dimension [x,y,z]
     * @param pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr&): incoming PCL point cloud pointer.
     * After appyling the filter, the caller already has filtered pointcloud at the same pointer.
     */
    static void apply_voxel_grid_filtering(
        const std::vector<float>& leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

    /**
     * It applies frustum culling filter on incoming point cloud.
     * Reference: https://pointclouds.org/documentation/classpcl_1_1_frustum_culling.html
     *
     * @param v_fov (const float): camera's vertical field of view
     * @param h_fov (const float): camera's horizontal field of view
     * @param near_plane_dist (const float): frustum's near plane
     * @param far_plane_dist (const far_plane_dist): frustum's far plane
     * @param pose (const Eigen::Matrix4f&): camera pose w.r.t. to the origin
     * @param pointcloud (pcl::PointCloud<pcl::PointXYZ>::Ptr&): incoming PCL point cloud pointer.
     * After appyling the filter, the caller already has filtered pointcloud at the same pointer.
     */
    static void apply_frustum_culling(
        const float v_fov, const float h_fov, const float near_plane_dist,
        const float far_plane_dist, const Eigen::Matrix4f& pose,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

    // upper bound of a depth value in meters (m)
    static constexpr float depth_upper_limit = 65.535;

    // lower bound of a depth value in meters (m)
    static constexpr float depth_lower_limit = .196;

private:
    // camera's intrinsics
    Eigen::Matrix3f intrinsics_;

    /**
     * This function takes 2D depth map and flattens the matrix with a single row vector
     * i.e. Given 2D depth map (H,W), a flattened row vector becomes (1,H*W)
     *
     * @param depthmap (const cv::Mat&) : 2D depth map with a size of (height, width)
     * @return (Eigen::MatrixXf) : (1,N) flattened matrix
     */
    Eigen::MatrixXf convert_depthmap_to_eigen_row_matrix(const cv::Mat& depthmap);
};

}  // namespace graph_slam
