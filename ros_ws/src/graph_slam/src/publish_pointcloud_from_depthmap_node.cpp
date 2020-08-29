#include <graph_slam/publish_pointcloud_from_depthmap_node.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace graph_slam
{

NodePublishPointcloudFromDepthmap::NodePublishPointcloudFromDepthmap(
    const std::string& in_odom_topic,
    const std::string& in_depthmap_topic,
    const std::string& out_cloud_topic,
    const Eigen::Matrix3Xf& intrinsics_matrix)
{
    ros::NodeHandle nh;

    // Subscribe and setup message sync
    odom_sub_.subscribe(nh, in_odom_topic, 5);
    depthmap_sub_.subscribe(nh, in_depthmap_topic, 5);

    sync_.reset(new Sync_(SyncPolicy_(10), odom_sub_, depthmap_sub_));
    sync_->registerCallback(
        boost::bind(
            &NodePublishPointcloudFromDepthmap::Callback, this, _1, _2));
    
    // Setup InverseProjection object
    inverse_projection_ =
        std::make_shared<InverseProjection>(intrinsics_matrix);

    // Setup Publishers
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 5);
}

NodePublishPointcloudFromDepthmap::~NodePublishPointcloudFromDepthmap()
     = default;

void NodePublishPointcloudFromDepthmap::Callback(
    const nav_msgs::OdometryConstPtr& odom_msg,
    const sensor_msgs::ImageConstPtr& depthmap_msg)
{
    // Create cloud from depthmap

    // Create ROS PointCloud2 message

    // Change PointCloud2's timestamp to odom's timestamp

    // Publish PointCloud2 message

}

}  // namespace graph_slam


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sync_odom_camera_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::string in_odom_topic = "";
    std::string in_depthmap_topic = "";
    std::string out_cloud_topic = "";

    int bad_params = 0;

    bad_params += !pnh.getParam("in_odom_topic", in_odom_topic);
    bad_params += !pnh.getParam("in_depthmap_topic", in_depthmap_topic);
    bad_params += !pnh.getParam("out_cloud_topic", out_cloud_topic);

    if (bad_params > 0)
    {
        std::cout << "One or mode parameters not set! Exiting." << std::endl;
        return 1;
    }

    Eigen::Matrix3Xf intrinsics_matrix;
    intrinsics_matrix << 851.67697279, 0.0, 628.07270979,
                         0.0, 855.55430548, 354.57494198,
                         0.0, 0.0, 1.0;

    graph_slam::NodePublishPointcloudFromDepthmap node_odom_throttle(
        in_odom_topic,
        in_depthmap_topic,
        out_cloud_topic,
        intrinsics_matrix);

    ros::spin();
    return 0;
}
