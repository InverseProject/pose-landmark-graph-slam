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

class NodePublishPointcloudFromDepthmap
{

public:
    NodePublishPointcloudFromDepthmap(
        const std::string& in_odom_topic, const std::string& in_depthmap_topic,
        const std::string& out_cloud_topic, const Eigen::Matrix3Xf& intrinsics_matrix,
        const std::vector<bool>& filter_flags);

    ~NodePublishPointcloudFromDepthmap();

    void Callback(
        const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::ImageConstPtr& depthmap_msg);

private:
    /**
     * This function applies different point cloud filters to remove noise and outliers.
     * @param pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr&): shared pcl point cloud pointer
     */
    void apply_filters(pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud);

    // Camera intrisincis
    Eigen::Matrix3Xf intrinsics_matrix_;

    // Flags to enable different filters
    // filter_flags_[0] -> statistical outlier removal
    // filter_flags_[1] -> approximate voxel grid
    // filter_flags_[2] -> voxel grid
    // filter_flags_[3] -> frustum culling
    std::vector<bool> filter_flags_;

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
