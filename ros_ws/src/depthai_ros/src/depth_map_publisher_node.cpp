#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "depthai_ros/depth_map_publisher_node.h"
#include <unordered_map>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <depthai_ros/BoundingBox.h>
#include <depthai_ros/detection.h>
#include <math.h>
#include <list>


namespace depthai_ros
{

DepthMapPublisherNode::DepthMapPublisherNode(
    const std::string& config_file_path, const std::string& depth_map_topic,
    const std::string& landmark_topic, const int rate) :
      config_file_path_(config_file_path),
      depth_map_topic_(depth_map_topic),
      landmark_topic_(landmark_topic)
{
    int lambda = 199 * 100;
    float sigma = 15 / 10;

    ros::NodeHandle nh;
    ros::Rate loop_rate(rate);

    // setup the publisher for depth map and landmarks detection
    depth_map_pub_ = nh.advertise<sensor_msgs::Image>(depth_map_topic_, 10);
    landmarks_pub_ = nh.advertise<depthai_ros::detection>(landmark_topic_, 10);
    // start the device and create the pipeline
    oak_.reset(new DepthAI::DepthAI("", config_file_path_, false));
 
    wls_filter_ = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    wls_filter_->setLambda(lambda);
    wls_filter_->setSigmaColor(sigma);
}

// Destroying OAK-D ptr
DepthMapPublisherNode::~DepthMapPublisherNode() { oak_->~DepthAI(); }

// Depth map and landmark publisher
void DepthMapPublisherNode::Publisher(uint8_t disparity_confidence_threshold)
{
    // setting the disparity confidence threshold
    oak_->send_disparity_confidence_threshold(disparity_confidence_threshold);
    std::list<std::shared_ptr<NNetPacket>> op_NNet_detections;
    std::cout << "publishing" << std::endl;

    // creating lut to convert to depth map
    std::vector<int> lut(256, 0);
    float focal_length = 1280 / (2.f * std::tan(71.86 / 2 / 180.f * std::acos(-1)));
    for (int i = 0; i < 256; ++i)
    {
        float z_m = (focal_length * 7.5 / i);
        lut[i] = std::min(65535.f, z_m * 10.f);
    }

    while (ros::ok())
    {
        cv::Mat rectified_right, filtered_disparity, disp_map;
        // Fetching the frames and NN objects from the oak-d
        oak_->get_streams(
            output_streams_, op_NNet_detections);  

        depthai_ros::detection detection_msg;

        std::list<std::shared_ptr<NNetPacket>>::iterator it;
        // iterating over list of NNetPacket and adding detections to ros message
        for (it = op_NNet_detections.begin(); it != op_NNet_detections.end(); ++it)
        {
            auto obj_detections = (*it)->getDetectedObjects();
            depthai_ros::detection temp_detection_msg;
            for (int i = 0; i < obj_detections->detection_count; ++i)
            {
                auto detection = obj_detections->detections[i];
                depthai_ros::BoundingBox object;
                object.label_id =  detection.label ;
                object.confidence = detection.confidence ;
                object.xmin =  detection.x_min ;
                object.ymin =  detection.y_min ;
                object.xmax =  detection.x_max ;
                object.ymax =  detection.y_max ;
                object.depth.x = detection.depth_x ;
                object.depth.y = detection.depth_y ;
                object.depth.z = detection.depth_z ;
                temp_detection_msg.detections.push_back(object);
            }
            detection_msg = temp_detection_msg;
        }

        // using wls_filter object to use rectified right 
        // and disparity to create filtered disparity
        cv::flip(*output_streams_["rectified_right"], rectified_right, 1);
        output_streams_["disparity"]->convertTo(disp_map, CV_16S);
        wls_filter_->filter(disp_map, rectified_right, filtered_disparity);

        cv::Mat depth_map(720, 1280, CV_16UC1);
        // converting filtered disparity to depth map 
        for (int i = 0; i < filtered_disparity.rows; ++i)
        {
            signed short* p = filtered_disparity.ptr<signed short>(i);
            unsigned short* q = depth_map.ptr<unsigned short>(i);
            for (int j = 0; j < filtered_disparity.cols; ++j)
            {
                q[j] = lut[p[j]];
            }
        }

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "OAK-D-right";

        detection_msg.header = header;
        sensor_msgs::ImagePtr depthmap_msg =
            cv_bridge::CvImage(header, sensor_msgs::image_encodings::TYPE_16UC1, depth_map)
                .toImageMsg();
        depth_map_pub_.publish(depthmap_msg);
        landmarks_pub_.publish(detection_msg);
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
    std::string landmark_topic = "";
    int disparity_confidence_threshold;
    int rate;

    int bad_params = 0;

    bad_params += !pnh.getParam("config_file_path", config_file_path);
    bad_params += !pnh.getParam("depth_map_topic", depth_map_topic);
    bad_params += !pnh.getParam("landmark_topic", landmark_topic);
    bad_params += !pnh.getParam("disparity_confidence_threshold", disparity_confidence_threshold);
    bad_params += !pnh.getParam("rate", rate);

    if (bad_params > 0)
    {
        std::cout << "One or more parameters not set! Exiting." << std::endl;
        return 1;
    }

    depthai_ros::DepthMapPublisherNode depth_publisher(
        config_file_path, depth_map_topic, landmark_topic, rate);
    depth_publisher.Publisher(disparity_confidence_threshold);

    return 0;
}
