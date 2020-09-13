#include <string>
#include "ros/ros.h"
#include "depthai/depthai_wrapper.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

using CV_mat_ptr = std::shared_ptr<cv::Mat>;

class DepthMapPublisher
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
    DepthMapPublisher(
        const std::string config_file_path, const std::string depth_map_topic,
        const std::string landmark_topic, const int rate);

    /**
     * Destructor
     * Deinit the device before stopping the ros node.
     */
    ~DepthMapPublisher();

    /**
     *  Publishes the depth_map over the given topic in the constructor
     */
    void publisher();

    /**
     *  Service to send the confidence threshold
     */
    // bool service_callback(oak_d_publisher::disparity_threshold::Request &req,
    //                                     oak_d_publisher::disparity_threshold::Response &res);

private:
    // Class Private Variables
    std::string _config_file_path;
    std::string _depth_map_topic;
    std::string _landmark_topic;
    std::string _disparity_service_name;
    ros::ServiceServer _disparity_threshold_srv;
    ros::Publisher _depth_map_pub;
    DepthAI* oak;
    std::unordered_map<std::string, CV_mat_ptr> output_streams;
};
