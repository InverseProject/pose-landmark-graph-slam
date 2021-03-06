#include <vector>
#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <graph_slam/publish_pointcloud_from_depthmap_node.h>

namespace graph_slam
{

NodePublishPointcloudFromDepthmap::NodePublishPointcloudFromDepthmap(
    const std::string& in_odom_topic, const std::string& in_depthmap_topic,
    const std::string& out_cloud_topic, const Eigen::Matrix3Xf& intrinsics_matrix,
    const std::vector<bool>& filter_flags, int subsample_factor) :
      filter_flags_(filter_flags),
      subsample_factor_(subsample_factor)
{
    ros::NodeHandle nh;

    // Subscribe and setup message sync
    odom_sub_.subscribe(nh, in_odom_topic, 5);
    depthmap_sub_.subscribe(nh, in_depthmap_topic, 5);

    sync_.reset(new Sync_(SyncPolicy_(10), odom_sub_, depthmap_sub_));
    sync_->registerCallback(
        boost::bind(&NodePublishPointcloudFromDepthmap::Callback, this, _1, _2));

    // Setup DepthmapToPointCloudConverter object
    depthmap_to_pc_converter_ = std::make_shared<DepthmapToPointCloudConverter>(intrinsics_matrix);

    // Setup Publishers
    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 5);
}

void NodePublishPointcloudFromDepthmap::Callback(
    const nav_msgs::OdometryConstPtr& odom_msg, const sensor_msgs::ImageConstPtr& depthmap_msg)
{
    // Convert ROS image to OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(depthmap_msg, sensor_msgs::image_encodings::TYPE_16UC1);
    }
    catch (const cv_bridge::Exception& e)
    {
        std::cout << "Unable to convert sensor_msg using cv_bridge. Details: " << e.what()
                  << std::endl;
        return;
    }

    // Change the image value type from uint_16 to float_32
    cv_ptr->image.convertTo(cv_ptr->image, CV_32F);

    // Create cloud from depthmap
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc_from_depth =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    depthmap_to_pc_converter_->get_pcl_pointcloud(cv_ptr->image, subsample_factor_, pc_from_depth);

    // Apply different filters to reduce noise and remove outliers
    ApplyFilters(pc_from_depth);

    // Create ROS PointCloud2 message
    sensor_msgs::PointCloud2Ptr points_to_publish = boost::make_shared<sensor_msgs::PointCloud2>();

    // Convert obtained point cloud to ROS PointCloud2 message
    pcl::toROSMsg(*pc_from_depth, *points_to_publish);
    points_to_publish->header.frame_id = depthmap_msg->header.frame_id;

    // Change PointCloud2's timestamp to odom's timestamp
    points_to_publish->header.stamp = odom_msg->header.stamp;

    // Publish PointCloud2 message
    cloud_pub_.publish(points_to_publish);
}

void NodePublishPointcloudFromDepthmap::ApplyFilters(
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pointcloud)
{
    if (filter_flags_[0])
    {
        DepthmapToPointCloudConverter::apply_statistical_outlier_removal_filtering(
            50, 1, pointcloud);
    }

    if (filter_flags_[1])
    {
        DepthmapToPointCloudConverter::apply_approximate_voxel_grid_filtering(
            {0.01f, 0.01f, 0.01f}, pointcloud);
    }

    if (filter_flags_[2])
    {
        DepthmapToPointCloudConverter::apply_voxel_grid_filtering(
            {0.01f, 0.01f, 0.01f}, pointcloud);
    }

    if (filter_flags_[3])
    {
        Eigen::Matrix4f cam_pose;

        // traditional camera pose (X->Right, Y->Down, Z->Forward)
        cam_pose << 1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        DepthmapToPointCloudConverter::apply_frustum_culling(
            56.74, 71.86, 1.0, 5.0, cam_pose, pointcloud);
    }
}

}  // namespace graph_slam

int main(int argc, char** argv)
{
    ros::init(argc, argv, "publish_pointcloud_from_depthmap_node");
    ros::NodeHandle pnh("~");

    std::string in_odom_topic = "";
    std::string in_depthmap_topic = "";
    std::string out_cloud_topic = "";
    std::vector<float> intrinsics;
    std::vector<bool> filter_flags;
    int subsample_factor;

    int bad_params = 0;

    bad_params += !pnh.getParam("in_odom_topic", in_odom_topic);
    bad_params += !pnh.getParam("in_depthmap_topic", in_depthmap_topic);
    bad_params += !pnh.getParam("out_cloud_topic", out_cloud_topic);
    bad_params += !pnh.getParam("intrinsic_matrix", intrinsics);
    bad_params += !pnh.getParam("filter_flags", filter_flags);
    bad_params += !pnh.getParam("subsample_factor", subsample_factor);

    if (bad_params > 0)
    {
        std::cout << "One or mode parameters not set! Exiting." << std::endl;
        return 1;
    }

    Eigen::Matrix3f intrinsics_matrix = Eigen::Map<Eigen::Matrix3f>(intrinsics.data());

    graph_slam::NodePublishPointcloudFromDepthmap node_odom_throttle(
        in_odom_topic, in_depthmap_topic, out_cloud_topic, intrinsics_matrix, filter_flags,
        subsample_factor);

    ros::spin();
    return 0;
}
