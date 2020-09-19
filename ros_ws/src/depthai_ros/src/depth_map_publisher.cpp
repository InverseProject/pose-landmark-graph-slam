#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "depthai_ros/depth_map_publisher.h"
// #include "oak_d_publisher/disparity_threshold.h"
#include <unordered_map>

namespace depthai_ros
{

    DepthMapPublisher::DepthMapPublisher(
        const std::string config_file_path, const std::string depth_map_topic,
        const std::string landmark_topic, const int rate) :
        config_file_path_(config_file_path),
        depth_map_topic_(depth_map_topic),
        landmark_topic_(landmark_topic)
    {

        ros::NodeHandle n;
        ros::Rate loop_rate(rate);

        // disparity_service_name_ = "/disparity_confidence_threshold";

        // setup the publisher for depth map
        depth_map_pub_ = n.advertise<sensor_msgs::Image>(depth_map_topic_, 10);

        // start the device and create the pipeline
        oak_.reset(new DepthAI("", config_file_path_, false));
        // disparity_threshold_srv_ = n.advertiseService(_disparity_service_name,
        // &DepthMapPublisher::service_callback, this);
    }

    // Destroying OAK-D ptr
    DepthMapPublisher::~DepthMapPublisher() { oak_->~DepthAI(); }

    // Service for setting depth threshold
    // bool DepthMapPublisher::service_callback(oak_d_publisher::disparity_threshold::Request &req,
    //                                         oak_d_publisher::disparity_threshold::Response &res){

    //     oak->send_disparity_confidence_threshold(req.threshold);
    //     res.is_set = true;
    //     return true;
    // }

    // Depth map publisher
    void DepthMapPublisher::Publisher()
    {
        cv_bridge::CvImage depth_msg;
        depth_msg.encoding = sensor_msgs::image_encodings::MONO16;
        while (ros::ok())
        {
            oak_->get_frames(output_streams_);  // Fetching the frames from the oak-d
            depth_msg.header.stamp = ros::Time::now();
            depth_msg.header.frame_id = "OAK-D-Right";
            depth_msg.image = *output_streams_["depth_raw"];
            depth_map_pub_.publish(depth_msg.toImageMsg());

            ros::spinOnce();
        }

        return;
    }

    int main(int argc, char** argv)
    {
        ros::init(argc, argv, "depth_map_publisher_node");

        ros::NodeHandle pnh("~");

        std::string config_file_path = "";
        std::string depth_map_topic = "";
        std::string landmark_topic = "";
        std::string rate_str = "";
        int rate;

        int bad_params = 0;

        bad_params += !pnh.getParam("config_file_path", config_file_path);
        bad_params += !pnh.getParam("depth_map_topic", depth_map_topic);
        bad_params += !pnh.getParam("landmark_topic", landmark_topic);
        bad_params += !pnh.getParam("rate", rate);

        if (bad_params > 0)
        {
            std::cout << "One or more parameters not set! Exiting." << std::endl;
            return 1;
        }

        DepthMapPublisher depth_publisher(config_file_path, depth_map_topic, landmark_topic, rate);
        depth_publisher.Publisher();

        return 0;
    }

}