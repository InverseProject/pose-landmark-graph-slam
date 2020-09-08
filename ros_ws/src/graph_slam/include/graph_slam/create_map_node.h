#pragma once

#include <iostream>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>

namespace graph_slam
{

class CreateMapNode
{
public:
    CreateMapNode(
        const std::string& in_odom_topic, const std::string& in_cloud_topic,
        const std::string& out_cloud_topic);

    ~CreateMapNode();

    void Callback(
        const nav_msgs::OdometryConstPtr& odom_msg,
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
    // Class variables
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_map_cloud_;

    // ROS Subscribers
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub_;
    message_filters::Subscriber<sensor_msgs::PointCloud2> cloud_sub_;
    std::shared_ptr<message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>>
        sync_;

    // ROS Publishers
    ros::Publisher map_cloud_pub_;

};  // class CreateMapNode

}  // namespace graph_slam
