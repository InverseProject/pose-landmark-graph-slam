#pragma once

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <string>
#include <vector>

#include <graph_slam/depthmap_to_pointcloud_converter.h>

namespace graph_slam
{

/**
 * ROS subscriber to test depthmap to point cloud conversion
 */
class DepthToPCTestNode
{

public:
    /**
     * Constructor
     *
     * @param in_depthmap_topic (const std::string&): ROS topic for raw depth maps
     * @param out_cloud_topic (const std::string&): ROS topic for converted point cloud
     * @param intrinsic_matrix (const Eigen::Matrix3Xf&): camera's intrinsics matrix
     * @param filter_flags (const std::vector<bool>&): flags for point cloud filters. Currently, we
     * only support 4 different filters.
     * filter_flags_[0] -> statistical outlier removal
     * filter_flags_[1] -> approximate voxel grid
     * filter_flags_[2] -> voxel grid
     * filter_flags_[3] -> frustum culling
     * @param subsample_factor (int): subsampling factor to skip over points for reducing total
     * number of points
     */
    DepthToPCTestNode(
        const std::string& in_depthmap_topic, const std::string& out_cloud_topic,
        const Eigen::Matrix3Xf& intrinsics_matrix, const std::vector<bool>& filter_flags,
        int subsample_factor);

    /**
     * Desturctor
     */
    ~DepthToPCTestNode() = default;

    /**
     * Callback function to read depth map and convert it into point cloud for publishing.
     * This function internally uses following private variables
     * -> std::vector<bool> filter_flags_
     * -> int subsample_factor_
     *
     * @param depthmap_msg (const sensor_msgs::ImageConstPtr&): Depth map ROS sensor message
     */
    void Callback(const sensor_msgs::ImageConstPtr& depthmap_msg);

private:
    /**
     * This function applies different point cloud filters to remove noise and outliers. This
     * function internally uses private boolean vector, filter_flags_.
     *
     * @param pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr&): shared pcl point cloud pointer
     */
    void ApplyFilters(pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

    // vector container to store filter flags
    std::vector<bool> filter_flags_;

    // subsampling factor
    int subsample_factor_ = 1;

    // ROS subscriber for depth map
    ros::Subscriber depthmap_sub_;

    // ROS publisher for pont cloud
    ros::Publisher cloud_pub_;

    // Depth to cloud converter object
    std::shared_ptr<DepthmapToPointCloudConverter> depthmap_to_pc_converter_;
};

}  // namespace graph_slam
