
#include <iostream>
#include <string>
#include "depthai/device.hpp"
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "depthai/depthai_wrapper.hpp"

class DepthMapPublisher
{

    public:
    DepthMapPublisher(
            const std::string& config_file_path,
            const std::string& depth_map_topic,
            const std::string& landmark_topic,
            const int rate);

    void publisher();


    private:
    using CV_mat_ptr = std::shared_ptr<cv::Mat>;
    std::string _config_file_path;
    std::string _depth_map_topic;
    std::string _landmark_topic;
    ros::Publisher _depth_map_pub;
    DepthAI *oak;
    std::unordered_map<std::string, CV_mat_ptr> output_streams;

};
