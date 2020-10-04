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
     */
    DepthToPCTestNode(
        const std::string& in_depthmap_topic, const std::string& out_cloud_topic,
        const Eigen::Matrix3Xf& intrinsics_matrix);

    /**
     * Desturctor
     */
    ~DepthToPCTestNode() = default;

    /**
     * Callback function to read depth map and convert it into point cloud for publishing
     *
     * @param depthmap_msg (const sensor_msgs::ImageConstPtr&): Depth map ROS sensor message
     */
    void Callback(const sensor_msgs::ImageConstPtr& depthmap_msg);

private:
    // ROS subscriber for depth map
    ros::Subscriber depthmap_sub_;

    // ROS publisher for pont cloud
    ros::Publisher cloud_pub_;

    // Depth to cloud converter object
    std::shared_ptr<DepthmapToPointCloudConverter> depthmap_to_pc_converter_;
};

}  // namespace graph_slam
