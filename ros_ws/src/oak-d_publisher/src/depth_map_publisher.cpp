
#include <iostream>
#include <string>
#include "depthai/device.hpp"
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "oak-d_publisher/depth_map_publisher.hpp"
#include <sensor_msgs/Image.h>

using namespace std;
    
DepthMapPublisher::DepthMapPublisher(
            const std::string& config_file_path,
            const std::string& depth_map_topic,
            const std::string& landmark_topic,
            const int rate):
            _config_file_path(config_file_path), 
            _depth_map_topic(depth_map_topic),
            _landmark_topic(landmark_topic){

    ros::init(argc, argv, "oak_publisher");
    ros::NodeHandle n;
    ros::Rate loop_rate(rate);

    std::ifstream file(_config_file_path);
    std::ostringstream file_stream;
    file_stream << file.rdbuf();
    std::string config_str = file_stream.str();

    Device oak("", true);
    _pipeline = oak.create_pipeline(config_str);

    _depth_map_pub = n.advertise<sensor_msgs::Image>(_depth_map_topic, 2);
}


DepthMapPublisher::publisher(){
    
    sensor_msgs::Image depth_map;
    depth_map.encoding = sensor_msgs::image_encodings::TYPE_16UC1;

    while(ros::ok()){

        _packets = pipeline->getAvailableNNetAndDataPackets(true);
        std::list<std::shared_ptr<HostDataPacket>> host_packet = get<1>(packets);
        
        for(auto sub_packet :  host_packet){
            std::cout << "Stream name ::------->" << sub_packet->stream_name << std::endl
            if(sub_packet->stream_name == "depth_raw"){

            }
         }

    }


}



int main(){


    return 0;
}