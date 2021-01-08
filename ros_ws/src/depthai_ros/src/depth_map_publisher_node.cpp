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


    bool outputDepth = true;
    bool outputRectified = false;
    bool lrcheck  = false;
    bool extended = false;
    bool subpixel = false;

    // Creating pipeline
    auto stereo = p_.create<dai::node::StereoDepth>();
    auto monoLeft = p_.create<dai::node::MonoCamera>();
    auto monoRight = p_.create<dai::node::MonoCamera>();
    auto colorCam = p_.create<dai::node::ColorCamera>();
    auto xlinkColorOut = p_.create<dai::node::XLinkOut>();
    auto xoutDepth = p_.create<dai::node::XLinkOut>();
    
    xlinkColorOut->setStreamName("video");
    xoutDepth->setStreamName("depth");
    // colorCam->setPreviewSize(300, 300);
    colorCam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    colorCam->setInterleaved(true);
    colorCam->setCamId(0);
    // Link plugins CAM -> XLINK
    colorCam->video.link(xlinkColorOut->input);

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setCamId(1);
    // monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setCamId(2);

    // StereoDepth
    stereo->setOutputDepth(outputDepth);
    stereo->setOutputRectified(outputRectified);
    stereo->setConfidenceThreshold(200);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout

    stereo->setLeftRightCheck(lrcheck);
    stereo->setExtendedDisparity(extended);
    stereo->setSubpixel(subpixel);
    // Link plugins CAM -> STEREO -> XLINK
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    stereo->depth.link(xoutDepth->input);
    std::cout << "sstarting decive -------> " <<std::endl;
    oak_.reset(new dai::Device(p_));
    oak_->startPipeline();
}

// Destroying OAK-D ptr
DepthMapPublisherNode::~DepthMapPublisherNode() { oak_->~Device(); }

// Depth map publisher
void DepthMapPublisherNode::Publisher(uint8_t disparity_confidence_threshold)
{
    // oak_->send_disparity_confidence_threshold(disparity_confidence_threshold);

    auto depthQueue = oak_->getOutputQueue("depth", 8, true);
    auto videoQueue = oak_->getOutputQueue("video");

    cv::Mat frame;
    
    while (ros::ok())
    {
        // oak_->get_streams(output_streams_);  // Fetching the frames from the oak-d
        // std::cout << "waaiting for stream -------> " <<std::endl;
        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "OAK-D-right";

        auto depth = depthQueue->get<dai::ImgFrame>();
        auto imgFrame = videoQueue->get<dai::ImgFrame>();

        if(imgFrame){
            frame = cv::Mat(imgFrame->getHeight() * 3 / 2, imgFrame->getWidth(), CV_8UC1, imgFrame->getData().data());
            cv::Mat rgb(imgFrame->getHeight(), imgFrame->getWidth(), CV_8UC3);
            cv::cvtColor(frame, rgb, cv::COLOR_YUV2BGR_NV12);
            // std::cout << "rgb frame size: " << rgb.size() << std::endl;
            sensor_msgs::ImagePtr color_msg =
                cv_bridge::CvImage(
                    header, sensor_msgs::image_encodings::BGR8, rgb)
                    .toImageMsg();

            rgb_pub_.publish(color_msg);
        }
        
        if(depth){
            sensor_msgs::ImagePtr depthmap_msg =
                cv_bridge::CvImage(
                    header, sensor_msgs::image_encodings::TYPE_16UC1,
                    cv::Mat(depth->getHeight(), depth->getWidth(), CV_16UC1, depth->getData().data()))
                    .toImageMsg();
            depth_map_pub_.publish(depthmap_msg);
        }
        

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
