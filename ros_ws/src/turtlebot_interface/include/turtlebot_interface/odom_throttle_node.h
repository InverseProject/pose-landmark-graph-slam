#pragma once

#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace turtlebot_interface
{

/**
 * Class for throttling incoming raw odometry messages. Only relays odometry 
 * messages which correspond to significant movement in the robot.
*/

class NodeOdometryThrottle
{

public:
    /**
     * Constructor
     * 
     * @param in_odom_topic (const std::string&): Input raw odometry topic
     * @param out_odom_topic (const std::string&): Ouput throttled odometry 
     * topic
     * @param distance_threshold (float): The minimum distance (in meters) that 
     * the robot must have traveled in order for odometry to be passed through.
     * @param rotation_threshold (float): The minimum yaw rotation (around Z-
     * axis) in radians the robot must have rotated in order for odometry to be
     * passed through.
    */
    NodeOdometryThrottle(
        const std::string& in_odom_topic,
        const std::string& out_odom_topic,
        float distance_threshold,
        float rotation_threshold);
    
    /**
     * Destructor
    */
    ~NodeOdometryThrottle();

    /**
     * Callback function that takes incoming odometry messages and only 
     * republishes messages that exceed movement thresholds
     * 
     * @param odom_msg (const nav_msgs::OdometryConstPtr&): Odometry message
    */
    void Callback(const nav_msgs::OdometryConstPtr& odom_msg);

private:

    // Input parameters
    float distance_threshold_ = 0.05; // in meters
    float rotation_threshold_ = 0.1; // in radians (~5 deg)

    // Current variables
    bool odom_published_at_least_once_ = false;
    nav_msgs::Odometry prev_odom_msg_published_;

    // Publishers
    ros::Publisher throttled_odom_pub_;

    // Subscribers
    ros::Subscriber odom_sub_;

    /**
     * This function checks if the current odometry message exceeds movement
     * thresholds (in translation and rotation) when compared to 
     * `prev_odom_msg_published_` object.
     * 
     * @param odom_msg (const nav_msgs::OdometryConstPtr&): Odometry message
     * @return (bool): True, if threshold exceeded, false otherwise.
    */
    bool ExceedsThresholds(const nav_msgs::OdometryConstPtr& odom_msg);

};

}  // namespace turtlebot_interface
