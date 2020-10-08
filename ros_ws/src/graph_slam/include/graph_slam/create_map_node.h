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

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace graph_slam
{

/**
 * Class for creating point cloud map using Odometry poses as the true poses of the robot. No
 * pose-graph optimization is currently performed in this Node.
 */
class CreateMapNode
{
public:
    /**
     * Constructor
     */
    CreateMapNode(
        const std::string& in_odom_topic, const std::string& in_cloud_topic,
        const std::string& out_cloud_topic, bool use_odom_as_correct_poses);

    /**
     * Destructor
     */
    ~CreateMapNode();

    /**
     * Callback function for creating map using odometry poses as true poses
     *
     * @param odom_msg (const nav_msgs::OdometryConstPtr&): Throttled odometry message to get poses.
     * These poses are treated as true poses for the robot.
     *
     * @param cloud_msg (const sensor_msgs::PointCloud2ConstPtr&): Point Cloud message. It is
     * assumed that these point clouds are synchronized with the odometry messages and share the
     * same timestamps.
     */
    void CallbackCreateMapWithOdomPoses(
        const nav_msgs::OdometryConstPtr& odom_msg,
        const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

private:
    // tf2 for lookup transform
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Class variables
    bool use_odom_as_correct_poses_ = true;
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
