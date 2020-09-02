#include <vector>
#include <graph_slam/publish_pointcloud_from_depthmap_node.h>
#include <ros/ros.h>
#include <Eigen/Dense>

namespace graph_slam
{

NodePublishPointcloudFromDepthmap::NodePublishPointcloudFromDepthmap(
    const std::string& in_odom_topic,
    const std::string& in_depthmap_topic,
    const std::string& out_cloud_topic,
    const Eigen::Matrix3Xf& intrinsics_matrix,
    int subsample_factor) :
    subsample_factor_(subsample_factor),
    intrinsics_matrix_(intrinsics_matrix)
{
    ros::NodeHandle nh;

    // Subscribe and setup message sync
    odom_sub_.subscribe(nh, in_odom_topic, 5);
    depthmap_sub_.subscribe(nh, in_depthmap_topic, 5);

    sync_.reset(new Sync_(SyncPolicy_(10), odom_sub_, depthmap_sub_));
    sync_->registerCallback(
        boost::bind(
            &NodePublishPointcloudFromDepthmap::Callback, this, _1, _2));
    
    // Setup DepthmapToPointCloudConverter object
    depthmap_to_pc_converter_ =
        std::make_shared<DepthmapToPointCloudConverter>(intrinsics_matrix);

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
    
    // depthmap_to_pc_converter_.get_pcl_pointcloud();

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
    int subsample_factor = 1;
    std::vector<float> intrinsics;

    int bad_params = 0;

    bad_params += !pnh.getParam("in_odom_topic", in_odom_topic);
    bad_params += !pnh.getParam("in_depthmap_topic", in_depthmap_topic);
    bad_params += !pnh.getParam("out_cloud_topic", out_cloud_topic);
    bad_params += !pnh.getParam("subsample_factor", subsample_factor);
    bad_params += !pnh.getParam("intrinsic_matrix", intrinsics);

    if (bad_params > 0)
    {
        std::cout << "One or mode parameters not set! Exiting." << std::endl;
        return 1;
    }

    Eigen::Matrix3f intrinsics_matrix = Eigen::Map<Eigen::Matrix3f>(intrinsics.data());

    graph_slam::NodePublishPointcloudFromDepthmap node_odom_throttle(
        in_odom_topic,
        in_depthmap_topic,
        out_cloud_topic,
        intrinsics_matrix,
        subsample_factor);

    ros::spin();
    return 0;
}
