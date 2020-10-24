#pragma once

#include <string>
#include "ros/ros.h"
#include "depthai/depthai_wrapper.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <unordered_map>
#include "opencv2/ximgproc.hpp"

using namespace cv;
using namespace cv::ximgproc;

namespace depthai_ros
{
using CV_mat_ptr = std::shared_ptr<cv::Mat>;

/**
 * DepthMapPublisher is a ROS node which launches OAK-D streams as specified in the config file.
 * Currently it is capable of publishing depth map produced by OAK-D.
 * Streaming AI based objects position is WIP
 */
class DepthMapPublisherNode
{

public:
    /**
     * Constructor
     * @param config_file_path (std::string): Provides the json file which is
     *  used by the Depthai to configure the OAK-D
     * @param depth_map_topic (std::string): Topic on which depth map will be broadcasted
     * @param landmark_topic (std::string): Topic on which landmarks map will be broadcasted. (WIP)
     * @param rate (int): Loop rate of ROS
     */
    DepthMapPublisherNode(
        const std::string& config_file_path, const std::string& depth_map_topic,
        const std::string& landmark_topic, const int rate);

    /**
     * Destructor
     * Stop the device before stopping the ROS node.
     */
    ~DepthMapPublisherNode();

    /**
     *  Publishes the depth_map over the given topic in the constructor
     * @param depth_confidence_threshold (uint8_t): disparity confidence threshold(0-255)
     * higher the confidence more the information is allowed.
     */
    void Publisher(uint8_t disparity_confidence_threshold);

private:
    // Class Private Variables
    std::string config_file_path_;
    std::string depth_map_topic_;
    std::string landmark_topic_;
    std::string disparity_service_name_;
    ros::Publisher depth_map_pub_;
    std::unique_ptr<DepthAI::DepthAI> oak_;
    std::unordered_map<std::string, CV_mat_ptr> output_streams_;
    Ptr<DisparityWLSFilter> wls_filter_;
};
}  // namespace depthai_ros
