#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "depthai_ros/depth_map_publisher_node.h"
#include <unordered_map>

namespace depthai_ros
{

DepthMapPublisherNode::DepthMapPublisherNode(
    const std::string& config_file_path, const std::string& depth_map_topic,
    const std::string& landmark_topic, const std::string& rgb_topic, const int rate) :
      config_file_path_(config_file_path),
      depth_map_topic_(depth_map_topic),
      rgb_topic_(rgb_topic),
      landmark_topic_(landmark_topic)
{

    ros::NodeHandle nh;
    ros::Rate loop_rate(rate);

    // setup the publisher for depth map
    depth_map_pub_ = nh.advertise<sensor_msgs::Image>(depth_map_topic_, 2);
    rgb_pub_ = nh.advertise<sensor_msgs::Image>(rgb_topic_, 2);
    // start the device and create the pipeline
    oak_.reset(new DepthAI::DepthAI("", config_file_path_, false));
}

// Destroying OAK-D ptr
DepthMapPublisherNode::~DepthMapPublisherNode() { oak_->~DepthAI(); }

// Depth map publisher
void DepthMapPublisherNode::Publisher(uint8_t disparity_confidence_threshold)
{
    oak_->send_disparity_confidence_threshold(disparity_confidence_threshold);

    while (ros::ok())
    {
        oak_->get_streams(output_streams_);  // Fetching the frames from the oak-d

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "OAK-D-right";

        sensor_msgs::ImagePtr depthmap_msg =
            cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::TYPE_16UC1, *output_streams_["depth"])
                .toImageMsg();

        sensor_msgs::ImagePtr color_msg =
            cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::BGR8, *output_streams_["color"])
                .toImageMsg();

        depth_map_pub_.publish(depthmap_msg);
        rgb_pub_.publish(color_msg);

        ros::spinOnce();
    }

    return;
}

}  // namespace depthai_ros

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_map_publisher_node");

    ros::NodeHandle pnh("~");

    std::string config_file_path = "";
    std::string depth_map_topic = "";
    std::string rgb_topic = "";
    std::string landmark_topic = "";
    int disparity_confidence_threshold;
    int rate;

    int bad_params = 0;

    bad_params += !pnh.getParam("config_file_path", config_file_path);
    bad_params += !pnh.getParam("depth_map_topic", depth_map_topic);
    bad_params += !pnh.getParam("rgb_topic", rgb_topic);
    bad_params += !pnh.getParam("landmark_topic", landmark_topic);
    bad_params += !pnh.getParam("disparity_confidence_threshold", disparity_confidence_threshold);
    bad_params += !pnh.getParam("rate", rate);

    if (bad_params > 0)
    {
        std::cout << "One or more parameters not set! Exiting." << std::endl;
        return 1;
    }

    depthai_ros::DepthMapPublisherNode depth_publisher(
        config_file_path, depth_map_topic, landmark_topic, rgb_topic, rate);
    depth_publisher.Publisher(disparity_confidence_threshold);

    return 0;
}
