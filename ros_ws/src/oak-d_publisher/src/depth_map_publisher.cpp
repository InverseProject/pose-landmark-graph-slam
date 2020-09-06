
#include <iostream>
#include <string>
#include "depthai/device.hpp"
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "oak-d_publisher/depth_map_publisher.hpp"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>


using namespace std;
    
DepthMapPublisher::DepthMapPublisher(
            const std::string& config_file_path,
            const std::string& depth_map_topic,
            const std::string& landmark_topic,
            const int rate):
            _config_file_path(config_file_path), 
            _depth_map_topic(depth_map_topic),
            _landmark_topic(landmark_topic){

    
    ros::NodeHandle n;
    ros::Rate loop_rate(rate);

    std::ifstream file(_config_file_path);
    std::ostringstream file_stream;
    file_stream << file.rdbuf();
    config_str = file_stream.str();
    _depth_map_pub = n.advertise<sensor_msgs::Image>(_depth_map_topic, 2);

    oak = new Device("", false);
    // std::cout << "Creating pipeline ---------------->" << std::endl << config_str << std::endl;
        _pipeline = oak->create_pipeline(config_str);

    // std::cout << "Created pipeline ---------------->" << endl;
}


void DepthMapPublisher::publisher(){
    
    cv_bridge::CvImage out_msg;
    out_msg.encoding = sensor_msgs::image_encodings::MONO16 ; // Or whatever
    // out_msg.image    = sal_float_image; // Your cv::Mat

    sensor_msgs::Image depth_map;
    depth_map.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    // vector<uint16_t> img_data(720*1280, 0);    
    // unsigned char* depth_data = reinterpret_cast<unsigned char*>(img_data.data());
    
    cv::Mat img_data = cv::Mat(720,1280 , CV_16U);
    unsigned char* depth_data = reinterpret_cast<unsigned char*>(img_data.data);
    
    while(ros::ok())
    {

        _packets = _pipeline->getAvailableNNetAndDataPackets(true);
        // std::cout << "<-------------- colelcted packers ----------> " << std::endl;
        std::list<std::shared_ptr<HostDataPacket>> host_packet = get<1>(_packets);
    
        for(auto sub_packet :  host_packet){
            if(sub_packet->stream_name == "depth_raw"){
                auto received_data = sub_packet->getData();
                memcpy(depth_data, received_data, sub_packet->size());
                out_msg.header.stamp = ros::Time::now();
                out_msg.header.frame_id = "depth_map";
                out_msg.image = img_data;
                _depth_map_pub.publish(out_msg.toImageMsg());
                ros::spinOnce();
            }
         }
    }


    

}



int main(int argc, char** argv){
    ros::init(argc, argv, "oak_publisher");

    DepthMapPublisher oak("/home/jetbot/opencv_comp/try_3/pose-landmark-graph-slam/ros_ws/src/oak-d_publisher/config/config.json", "depth_map", "landmark", 10);
    // std::cout << "Starting publishing" << endl;
    oak.publisher();
 /*       
    std::string config = "{streams: [left, right],}";

    std::ifstream myFile("/home/jetbot/opencv_comp/try_3/pose-landmark-graph-slam/ros_ws/src/oak-d_publisher/config/config.json");
    std::ostringstream tmp;
    tmp << myFile.rdbuf();
    config = tmp.str();


    Device d("", true);
    cout << "------------found---------" << endl;
    cout << config << endl;
    std::shared_ptr<CNNHostPipeline> pipeline = d.create_pipeline(config);

    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>> packets; 
        int i = 0;

    // vector<uint16_t> img_data(720*1280, 0);    
    cv::Mat img_data = cv::Mat(720,1280 , CV_16U);
    unsigned char* depth_data = reinterpret_cast<unsigned char*>(img_data.data);

    while(true){
            packets = pipeline->getAvailableNNetAndDataPackets(true);
            std::list<std::shared_ptr<HostDataPacket>> host_packet = get<1>(packets);
            for(auto sub_packet :  host_packet){
                    cout << sub_packet->stream_name <<"::---------------::" << sub_packet->size() <<endl;
            // reinterpret_cast<unsigned char*>(img_data.data()) 
                auto data = sub_packet->getData();

                // cout << typeid(*data).name() << endl;
                // unsigned char* x = reinterpret_cast<unsigned char*>(img_data.data());
                // reinterpret_cast<unsigned char*>(img_data.data()) =  data;
                memcpy ( depth_data, data, sub_packet->size());

                }
        i++;
        if(i == 10)
        break;
    }

    int wait;
    cin >> wait;
  */
    return 0;
}
