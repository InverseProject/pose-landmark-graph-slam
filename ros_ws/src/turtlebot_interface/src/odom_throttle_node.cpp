
#include <turtlebot_interface/odom_throttle_node.h>

namespace turtlebot_interface
{

NodeOdometryThrottle::NodeOdometryThrottle(
    const std::string& in_odom_topic,
    const std::string& out_odom_topic,
    float distance_threshold,
    float rotation_threshold) :
    distance_threshold_(distance_threshold),
    rotation_threshold_(rotation_threshold)
{
    ros::NodeHandle nh;

    // Setup subscriber
    odom_sub_ = nh.subscribe(in_odom_topic, 5, &NodeOdometryThrottle::Callback, this);

    // Setup publisher
    throttled_odom_pub_ = nh.advertise<nav_msgs::Odometry>(out_odom_topic, 5);
}

    
}  // namespace turtlebot_interface

int main(int argc, char** argv)
{
    ros::init(argc, argv, "odom_throttle_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string in_odom_topic = "";
    std::string out_odom_topic = "";
    float distance_threshold = 0.05;
    float rotation_threshold = 0.1;

    int bad_params = 0;

    bad_params += !pnh.getParam("in_odom_topic", in_odom_topic);
    bad_params += !pnh.getParam("out_odom_topic", out_odom_topic);
    bad_params += !pnh.getParam("distance_threshold", distance_threshold);
    bad_params += !pnh.getParam("rotation_threshold", rotation_threshold);

    if (bad_params > 0)
    {
        std::cout << "One or mode parameters not set! Exiting." << std::endl;
        return 1;
    }

    turtlebot_interface::NodeOdometryThrottle node_odom_throttle(
        in_odom_topic,
        out_odom_topic,
        distance_threshold,
        rotation_threshold);
    
    ros::spin();
    return 0;
}