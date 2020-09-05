
#include <iostream>
#include <string>
#include "depthai/device.hpp"
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"

class DepthMapPublisher
{

    public:
    DepthMapPublisher(
            const std::string& config_file_path,
            const std::string& depth_map_topic,
            const std::string& landmark_topic,
            const int rate);

//    ~DepthMapPublisher();

    // create_pipeline();

    void publisher();


    private:
    using PacketsTuple = std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>>;
    
    PacketsTuple packets;
    std::string _config_file_path;
    std::string _depth_map_topic;
    std::string _landmark_topic;
    std::string config_str;
    ros::Publisher _depth_map_pub;
    Device *oak;
    std::shared_ptr<CNNHostPipeline> _pipeline;
    std::tuple<
        std::list<std::shared_ptr<NNetPacket>>,
        std::list<std::shared_ptr<HostDataPacket>>> _packets;
};
