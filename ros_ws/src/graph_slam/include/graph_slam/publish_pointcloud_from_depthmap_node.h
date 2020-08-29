#pragma once

#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <graph_slam/inverse_perspective_projection.h>

namespace graph_slam
{

class NodePublishPointcloudFromDepthmap
{

public:
    NodePublishPointcloudFromDepthmap(
        const std::string& in_odom_topic,
        const std::string& in_depthmap_topic,
        const std::string& out_cloud_topic,
        const Eigen::Matrix3Xf& intrinsics_matrix);

    ~NodePublishPointcloudFromDepthmap();

    void Callback(
        const nav_msgs::OdometryConstPtr& odom_msg,
        const sensor_msgs::ImageConstPtr& depthmap_msg);

private:

    // Setup subscribers
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depthmap_sub_;

    typedef message_filters::sync_policies::ApproximateTime<
        nav_msgs::Odometry,
        sensor_msgs::Image> SyncPolicy_;
    
    typedef message_filters::Synchronizer<SyncPolicy_> Sync_;
    std::shared_ptr<Sync_> sync_;

    // Depth to cloud converter object
    std::shared_ptr<InverseProjection> inverse_projection_;

    // Setup publisher
    ros::Publisher cloud_pub_;
};
    
}  // namespace graph_slam
