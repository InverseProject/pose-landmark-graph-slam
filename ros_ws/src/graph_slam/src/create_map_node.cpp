#include <graph_slam/create_map_node.h>

#include <pcl_conversions/pcl_conversions.h>

namespace graph_slam
{

CreateMapNode::CreateMapNode(
    const std::string& in_odom_topic, const std::string& in_cloud_topic,
    const std::string& out_cloud_topic)
{
    ros::NodeHandle nh;

    // Setup subscribers
    odom_sub_.subscribe(nh, in_odom_topic, 5);
    cloud_sub_.subscribe(nh, in_cloud_topic, 5);

    sync_ = std::make_shared<
        message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>>(
        odom_sub_, cloud_sub_, 5);

    sync_->registerCallback(boost::bind(&CreateMapNode::Callback, this, _1, _2));

    // Setup publisher
    map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 5);

    // Initialize map cloud
    curr_map_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
}

CreateMapNode::~CreateMapNode() = default;

void CreateMapNode::Callback(
    const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Get Point cloud from cloud_msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *curr_cloud);

    // Create transform from Odometry message
    Eigen::Vector3f translation(
        odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y,
        odom_msg->pose.pose.position.z);

    Eigen::Quaternionf rotation(
        odom_msg->pose.pose.orientation.w, odom_msg->pose.pose.orientation.x,
        odom_msg->pose.pose.orientation.y, odom_msg->pose.pose.orientation.z);

    // Transform current cloud into the odometry frame
    pcl::transformPointCloud(*curr_cloud, *curr_cloud, translation, rotation);

    // Concatenate current cloud into the map cloud
    *curr_map_cloud_ += *curr_cloud;

    // Convert Point Cloud to ROS message
    sensor_msgs::PointCloud2Ptr out_cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*curr_map_cloud_, *out_cloud_msg);

    // Publish cloud
    map_cloud_pub_.publish(curr_map_cloud_);
}

}  // namespace graph_slam

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_map_node");
    ros::NodeHandle pnh("~");

    std::string in_odom_topic = "";
    std::string in_cloud_topic = "";
    std::string out_cloud_topic = "";

    int bad_params = 0;

    bad_params += !pnh.getParam("in_odom_topic", in_odom_topic);
    bad_params += !pnh.getParam("in_cloud_topic", in_cloud_topic);
    bad_params += !pnh.getParam("out_cloud_topic", out_cloud_topic);

    if (bad_params != 0)
    {
        std::cout << "One or more parameters are not set! Exiting." << std::endl;
        return 1;
    }

    graph_slam::CreateMapNode cmn(in_odom_topic, in_cloud_topic, out_cloud_topic);
    ros::spin();

    return 0;
}
