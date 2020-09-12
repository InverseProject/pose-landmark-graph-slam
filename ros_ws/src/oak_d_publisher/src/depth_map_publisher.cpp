
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
// #include "depthai/depthai_wrapper.hpp"
#include "oak_d_publisher/depth_map_publisher.hpp"
#include "oak_d_publisher/disparity_threshold.h"


using namespace std;
    
DepthMapPublisher::DepthMapPublisher(
            const std::string config_file_path,
            const std::string depth_map_topic,
            const std::string landmark_topic,
            const int rate):
            _config_file_path(config_file_path), 
            _depth_map_topic(depth_map_topic),
            _landmark_topic(landmark_topic){
    
    ros::NodeHandle n;
    ros::Rate loop_rate(rate);
    _disparity_service_name = "disparity_confidence_threshold";

    _depth_map_pub = n.advertise<sensor_msgs::Image>(_depth_map_topic, 2);
    oak = new DepthAI("", config_file_path, false);
    _disparity_threshold_srv = n.advertiseService(_disparity_service_name, &DepthMapPublisher::service_callback, this);
// &NumberCounter::callback_reset_counter
}



bool DepthMapPublisher::service_callback(oak_d_publisher::disparity_threshold::Request &req, 
                                        oak_d_publisher::disparity_threshold::Response &res){

    oak->send_disparity_confidence_threshold(req.threshold);
    res.is_set = true;
    return true;
}

void DepthMapPublisher::publisher(){
    
    cv_bridge::CvImage depth_msg;
    depth_msg.encoding = sensor_msgs::image_encodings::MONO16 ; // Or whatever
    
    // sensor_msgs::Image depth_map;
    // depth_map.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    
    // cv::Mat img_data = cv::Mat(720,1280 , CV_16U);
    // unsigned char* depth_data = reinterpret_cast<unsigned char*>(img_data.data);
    
    while(ros::ok())
    {

        // std::cout << "<-------------- colelcted packers ----------> " << std::endl;
        oak->get_frames(output_streams);
        depth_msg.header.stamp = ros::Time::now();
        depth_msg.header.frame_id = "depth_map";
        depth_msg.image = *output_streams["depth_raw"];
        _depth_map_pub.publish(depth_msg.toImageMsg());

        ros::spinOnce();
    }

    return;
    

}



int main(int argc, char** argv){
    ros::init(argc, argv, "oak_publisher");

    DepthMapPublisher depth_publisher("/home/jetbot/opencv_comp/try_3/pose-landmark-graph-slam/ros_ws/src/oak-d_publisher/config/config.json", "depth_map", "landmark", 10);
    // std::cout << "Starting publishing" << endl;
    depth_publisher.publisher();
 
    return 0;
}
