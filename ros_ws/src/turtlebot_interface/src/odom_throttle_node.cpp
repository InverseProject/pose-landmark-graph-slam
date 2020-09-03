
#include <Eigen/Dense>
#include <turtlebot_interface/odom_throttle_node.h>

namespace turtlebot_interface
{

NodeOdometryThrottle::NodeOdometryThrottle(
    const std::string& in_odom_topic, const std::string& out_odom_topic, float distance_threshold,
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

NodeOdometryThrottle::~NodeOdometryThrottle() = default;

void NodeOdometryThrottle::Callback(const nav_msgs::OdometryConstPtr& odom_msg)
{
    if (!odom_published_at_least_once_)
    {
        prev_odom_msg_published_ = *odom_msg;
        throttled_odom_pub_.publish(odom_msg);
        odom_published_at_least_once_ = true;
        return;
    }

    if (ExceedsThresholds(odom_msg))
    {
        prev_odom_msg_published_ = *odom_msg;
        throttled_odom_pub_.publish(odom_msg);
    }
}

bool NodeOdometryThrottle::ExceedsThresholds(const nav_msgs::OdometryConstPtr& odom_msg)
{
    // Compare poses
    Eigen::Vector3f prev_pose, curr_pose;

    curr_pose << odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z;

    prev_pose << prev_odom_msg_published_.pose.pose.position.x,
        prev_odom_msg_published_.pose.pose.position.y,
        prev_odom_msg_published_.pose.pose.position.z;

    float distance = (curr_pose - prev_pose).norm();
    if (distance >= distance_threshold_)
    {
        return true;
    }

    // Compare rotations
    Eigen::Quaternionf curr_quat = Eigen::Quaternionf(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

    Eigen::Quaternionf prev_quat = Eigen::Quaternionf(
        prev_odom_msg_published_.pose.pose.orientation.w,
        prev_odom_msg_published_.pose.pose.orientation.x,
        prev_odom_msg_published_.pose.pose.orientation.y,
        prev_odom_msg_published_.pose.pose.orientation.z);

    Eigen::Vector3f curr_rotation_angles = curr_quat.toRotationMatrix().eulerAngles(0, 1, 2);
    Eigen::Vector3f prev_rotation_angles = prev_quat.toRotationMatrix().eulerAngles(0, 1, 2);

    // Compare Yaw angle
    float yaw_rotation = std::fabs(curr_rotation_angles[2] - prev_rotation_angles[2]);
    if (yaw_rotation >= rotation_threshold_)
    {
        return true;
    }

    return false;
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
        in_odom_topic, out_odom_topic, distance_threshold, rotation_threshold);

    ros::spin();
    return 0;
}
