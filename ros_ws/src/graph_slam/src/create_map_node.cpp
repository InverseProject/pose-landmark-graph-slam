
#include <geometry_msgs/TransformStamped.h>
#include <pcl_conversions/pcl_conversions.h>

#include <graph_slam/create_map_node.h>

namespace graph_slam
{

CreateMapNode::CreateMapNode(
    const std::string& in_odom_topic, const std::string& in_cloud_topic,
    const std::string& out_cloud_topic, bool use_odom_as_correct_poses) :
      use_odom_as_correct_poses_(use_odom_as_correct_poses)
{
    ros::NodeHandle nh;

    // Setup subscribers
    odom_sub_.subscribe(nh, in_odom_topic, 5);
    cloud_sub_.subscribe(nh, in_cloud_topic, 5);

    sync_ = std::make_shared<
        message_filters::TimeSynchronizer<nav_msgs::Odometry, sensor_msgs::PointCloud2>>(
        odom_sub_, cloud_sub_, 5);

    if (use_odom_as_correct_poses_)
    {
        sync_->registerCallback(
            boost::bind(&CreateMapNode::CallbackCreateMapWithOdomPoses, this, _1, _2));
    }
    else
    {
        // TODO(deepak): Create Callback that uses optimized poses
    }

    // Setup publisher
    map_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 5);

    // Initialize map cloud
    curr_map_cloud_ = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

CreateMapNode::~CreateMapNode() = default;

void CreateMapNode::CallbackCreateMapWithOdomPoses(
    const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Get Point cloud from cloud_msg
    pcl::PointCloud<pcl::PointXYZ>::Ptr curr_cloud =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    pcl::fromROSMsg(*cloud_msg, *curr_cloud);

    // Getting translation and rotation from odom to OAK-D
    geometry_msgs::TransformStamped odom_to_oak_transform_stamped_msg;
    try
    {
        odom_to_oak_transform_stamped_msg =
            tf_buffer_->lookupTransform("odom", "OAK-D-right", ros::Time(cloud_msg->header.stamp));
    }
    catch (const tf2::TransformException& e)
    {
        std::cout << "Transform exception: " << e.what() << std::endl;
        return;
    }

    tf2::Transform odom_to_oak_transform;
    tf2::fromMsg(odom_to_oak_transform_stamped_msg.transform, odom_to_oak_transform);
    tf2::Vector3 odom_to_oak_origin_tf2 = odom_to_oak_transform.getOrigin();
    tf2::Quaternion odom_to_oak_quat_tf2 = odom_to_oak_transform.getRotation();
    Eigen::Vector3f odom_to_oak_origin = {static_cast<float>(odom_to_oak_origin_tf2.getX()),
                                          static_cast<float>(odom_to_oak_origin_tf2.getY()),
                                          static_cast<float>(odom_to_oak_origin_tf2.getZ())};
    Eigen::Quaternionf odom_to_oak_quat = {static_cast<float>(odom_to_oak_quat_tf2.getW()),
                                           static_cast<float>(odom_to_oak_quat_tf2.getX()),
                                           static_cast<float>(odom_to_oak_quat_tf2.getY()),
                                           static_cast<float>(odom_to_oak_quat_tf2.getZ())};

    // Transform current cloud into the odometry frame
    pcl::transformPointCloud(*curr_cloud, *curr_cloud, odom_to_oak_origin, odom_to_oak_quat);

    // Concatenate current cloud into the map cloud
    *curr_map_cloud_ += *curr_cloud;

    // Convert Point Cloud to ROS message
    sensor_msgs::PointCloud2Ptr out_cloud_msg = boost::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*curr_map_cloud_, *out_cloud_msg);
    out_cloud_msg->header.frame_id = odom_msg->header.frame_id;
    out_cloud_msg->header.stamp = cloud_msg->header.stamp;

    // Publish cloud
    map_cloud_pub_.publish(out_cloud_msg);
}

}  // namespace graph_slam

int main(int argc, char** argv)
{
    ros::init(argc, argv, "create_map_node");
    ros::NodeHandle pnh("~");

    std::string in_odom_topic = "";
    std::string in_cloud_topic = "";
    std::string out_cloud_topic = "";
    bool use_odom_as_correct_poses = true;

    int bad_params = 0;

    bad_params += !pnh.getParam("in_odom_topic", in_odom_topic);
    bad_params += !pnh.getParam("in_cloud_topic", in_cloud_topic);
    bad_params += !pnh.getParam("out_cloud_topic", out_cloud_topic);
    bad_params += !pnh.getParam("use_odom_as_correct_poses", use_odom_as_correct_poses);

    if (bad_params != 0)
    {
        std::cout << "One or more parameters are not set! Exiting." << std::endl;
        return 1;
    }

    graph_slam::CreateMapNode cmn(
        in_odom_topic, in_cloud_topic, out_cloud_topic, use_odom_as_correct_poses);
    ros::spin();

    return 0;
}
