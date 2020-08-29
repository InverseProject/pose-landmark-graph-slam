#pragma once

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace turtlebot_interface
{

class NodeOdometryThrottle
{
public:
    NodeOdometryThrottle(
        const std::string& in_odom_topic,
        const std::string& out_odom_topic,
        float distance_threshold,
        float rotation_threshold);
    
    ~NodeOdometryThrottle();

    void Callback(const nav_msgs::OdometryConstPtr& odom_msg);

private:

    // Input parameters
    float distance_threshold_ = 0.05; // in meters
    float rotation_threshold_ = 0.1; // in radians (~ 5 deg)

    // Current variables
    bool odom_published_at_least_once_ = false;
    nav_msgs::Odometry prev_odom_msg_published_;

    // Publishers
    ros::Publisher throttled_odom_pub_;

    // Subscribers
    ros::Subscriber odom_sub_;

    // Helper functions
    bool ExceedsThresholds(const nav_msgs::OdometryConstPtr& odom_msg);

};

}  // namespace turtlebot_interface
