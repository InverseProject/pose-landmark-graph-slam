#include <string>
#include <vector>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>

#include <graph_slam/depth_to_pc_test_node.h>

namespace graph_slam
{

DepthToPCTestNode::DepthToPCTestNode(
    const std::string& in_depthmap_topic, const std::string& out_cloud_topic,
    const Eigen::Matrix3Xf& intrinsics_matrix)
{
    ros::NodeHandle nh;

    depthmap_sub_ = nh.subscribe(in_depthmap_topic, 5, &DepthToPCTestNode::Callback, this);

    depthmap_to_pc_converter_ = std::make_shared<DepthmapToPointCloudConverter>(intrinsics_matrix);

    cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(out_cloud_topic, 5);
}

void DepthToPCTestNode::Callback(const sensor_msgs::ImageConstPtr& depthmap_msg)
{
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

    depthmap_to_pc_converter_->get_pcl_pointcloud(cv_ptr->image, pc_from_depth);

    // Apply different filters on point clouds
    DepthmapToPointCloudConverter::apply_statistical_outlier_removal_filtering(
        100, 1, pc_from_depth);

    // Create ROS PointCloud2 message
    sensor_msgs::PointCloud2Ptr points_to_publish = boost::make_shared<sensor_msgs::PointCloud2>();

    // Convert obtained point cloud to ROS PointCloud2 message
    pcl::toROSMsg(*pc_from_depth, *points_to_publish);
    points_to_publish->header.frame_id = depthmap_msg->header.frame_id;

    // Publish PointCloud2 message
    cloud_pub_.publish(points_to_publish);
}

}  // namespace graph_slam

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_pc_test_node");
    ros::NodeHandle pnh("~");

    std::vector<float> intrinsics;
    std::string in_depthmap_topic = "";
    std::string out_cloud_topic = "";

    int bad_params = 0;

    bad_params += !pnh.getParam("in_depthmap_topic", in_depthmap_topic);
    bad_params += !pnh.getParam("out_cloud_topic", out_cloud_topic);
    bad_params += !pnh.getParam("intrinsic_matrix", intrinsics);

    if (bad_params > 0)
    {
        std::cout << "Setting one or more parameters failed. Program exiting." << std::endl;
        return 1;
    }

    Eigen::Matrix3f intrinsics_eigen = Eigen::Map<Eigen::Matrix3f>(intrinsics.data());

    graph_slam::DepthToPCTestNode node_depth_to_pc(
        in_depthmap_topic, out_cloud_topic, intrinsics_eigen);

    ros::spin();

    return 0;
}
