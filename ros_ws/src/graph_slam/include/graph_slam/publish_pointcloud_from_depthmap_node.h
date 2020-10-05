#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <graph_slam/depthmap_to_pointcloud_converter.h>

namespace graph_slam
{

/**
 * This class takes care of publishing time synchronized point clouds given throttled odometry and
 * raw depth maps
 */
class NodePublishPointcloudFromDepthmap
{

public:
    /**
     * Constructor
     *
     * @param in_odom_topic (const std::string&): throttled odometry
     * @param in_depthmap_topic (const std::string&): raw depth map
     * @param out_cloud_topic (const std::string&): time synchronized point cloud
     * @param intrinsics_matrix (const Eigen::Matrix3Xf&): camera's intrinsics matrix
     * @param filter_flags (const std::vector<bool>&): flags for point cloud filters. Currently, we
     * only support 4 different filters.
     * filter_flags_[0] -> statistical outlier removal
     * filter_flags_[1] -> approximate voxel grid
     * filter_flags_[2] -> voxel grid
     * filter_flags_[3] -> frustum culling
     * @param subsample_factor (int):
     */
    NodePublishPointcloudFromDepthmap(
        const std::string& in_odom_topic, const std::string& in_depthmap_topic,
        const std::string& out_cloud_topic, const Eigen::Matrix3Xf& intrinsics_matrix,
        const std::vector<bool>& filter_flags, int subsample_factor);

    /**
     * Destructor
     */
    ~NodePublishPointcloudFromDepthmap() = default;

    /**
     * Callback fuction that takes both throttled odometry and raw depth map messages and publishes
     * time synchronized point clouds
     *
     * @param odom_msg (const nav_msgs::OdometryConstPtr&): odometry message
     * @param depthmap_msg (const sensor_msgs::ImageConstPtr&): depth map message
     */
    void Callback(
        const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::ImageConstPtr& depthmap_msg);

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

    // Setup subscribers
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depthmap_sub_;

    typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::Image>
        SyncPolicy_;

    typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
    std::shared_ptr<Sync_> sync_;

    // Depth to cloud converter object
    std::shared_ptr<DepthmapToPointCloudConverter> depthmap_to_pc_converter_;

    // Setup publisher
    ros::Publisher cloud_pub_;
};

}  // namespace graph_slam
