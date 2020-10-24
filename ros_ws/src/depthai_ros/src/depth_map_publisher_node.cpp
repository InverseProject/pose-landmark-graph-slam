#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include "depthai_ros/depth_map_publisher_node.h"
#include <unordered_map>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <math.h>


namespace depthai_ros
{

int lambda = 199*100;
float sigma = 13/10; 
int sigma_slider = 15;

static void on_trackbar_signma(int value, void*)
{
   sigma = value / float(10);
   return;
}

static void on_trackbar_lambda(int value, void*)
{
   lambda = value * 100;
   return;
}

static std::string type2str(int type) {
  std::string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

DepthMapPublisherNode::DepthMapPublisherNode(
    const std::string& config_file_path, const std::string& depth_map_topic,
    const std::string& landmark_topic, const int rate) :
      config_file_path_(config_file_path),
      depth_map_topic_(depth_map_topic),
      landmark_topic_(landmark_topic)
{

    ros::NodeHandle nh;
    ros::Rate loop_rate(rate);

    // setup the publisher for depth map
    depth_map_pub_ = nh.advertise<sensor_msgs::Image>(depth_map_topic_, 10);

    // start the device and create the pipeline
    oak_.reset(new DepthAI::DepthAI("", config_file_path_, false));
    wls_filter_ = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    wls_filter_->setLambda(lambda);
    wls_filter_->setSigmaColor(sigma);
    std::string name = "colored view rectified disparity";
    namedWindow(name, WINDOW_AUTOSIZE);
    // createTrackbar( "sigma", name, &sigma_slider, 100, on_trackbar_signma );
    // createTrackbar( "lambda", name, &lambda, 255, on_trackbar_lambda );

}

// Destroying OAK-D ptr
DepthMapPublisherNode::~DepthMapPublisherNode() { oak_->~DepthAI(); }

// Depth map publisher
void DepthMapPublisherNode::Publisher(uint8_t disparity_confidence_threshold)
{
    oak_->send_disparity_confidence_threshold(disparity_confidence_threshold);

    while (ros::ok())
    {   
        cv::Mat rectified_right, filtered_disparity;
        oak_->get_streams(output_streams_);  // Fetching the frames from the oak-d
        
        cv::flip(*output_streams_["rectified_right"], rectified_right, 1);
        // wls_filter_->filter(*output_streams_["disparity"], rectified_right, filtered_disparity);
        
        // cv::Mat im_color_disp;
        // cv::applyColorMap(*output_streams_["disparity"], im_color_disp, cv::COLORMAP_JET);
        // cv::imshow("direct disparity",im_color_disp );
        // cv::waitKey(1);
        double min, max;
        // cv::minMaxLoc(*output_streams_["disparity"], &min, &max);
        // std::cout << min << "   " << max << std::endl;
        // std::cout << "Field vales" << std::endl;
        // for(auto kv : output_streams_){
        //     std::cout << kv.first << std::endl;
        // }    
        cv::Mat disp_map;
        output_streams_["disparity"]->convertTo(disp_map, CV_16S);
        // wls_filter_->setLambda(lambda);
        // wls_filter_->setSigmaColor(sigma);
    
        wls_filter_->filter(disp_map, rectified_right, filtered_disparity);

        // cv::minMaxLoc(disp_map, &min, &max);
        // std::cout << min << "   " << max << std::endl;
        // cv::imshow("rectified right", rectified_right);
        // cv::waitKey(1);
        // // cv::imshow("direct view", img);
        // // cv::waitKey(1);

        cv::Mat im_color;
        filtered_disparity.convertTo(im_color, CV_8U);
        cv::applyColorMap(im_color, im_color, cv::COLORMAP_JET);
        cv::imshow("colored view rectified disparity", im_color);
        cv::waitKey(1);
        
        cv::minMaxLoc(filtered_disparity, &min, &max);
        std::cout << min << "   " << max << std::endl;

        std::vector<int> lut(256, 0);
        float focal_length = 1280 / (2.f * std::tan(71.86 / 2 / 180.f * std::acos(-1)));
        for (int i = 0; i < 256; ++i)
        {
            float z_m = ( focal_length * 7.5 / i);
            lut[i] = std::min(65535.f, z_m * 10.f); 
            std::cout << lut[i] << "--" << z_m* 10.f << "--" << i << std::endl; 
        }
        std::cout << type2str(filtered_disparity.type()) << std::endl;
        // throw std::runtime_error("Config file could not be found at ");
        cv::Mat depth_map(720, 1280, CV_16UC1);

        for(int i = 0; i < filtered_disparity.rows; ++i)
        {
            signed short* p = filtered_disparity.ptr<signed short>(i);
            unsigned short* q = depth_map.ptr<unsigned short>(i);
            for (int j = 0; j < filtered_disparity.cols; ++j)
            {  
                //  std::cout << filtered_disparity[i][j] << std::endl;
                q[j] = lut[p[j]];
            }
        }
        cv::minMaxLoc(depth_map, &min, &max);
        std::cout << min << "   " << max << std::endl;
        cv::imshow("colored view rectified depth", depth_map);
        cv::Mat im_colored;
        depth_map.convertTo(im_colored, CV_8U);
        cv::applyColorMap(im_colored, im_colored, cv::COLORMAP_JET);
        cv::imshow("colored view rectified depth2", im_colored);
        cv::waitKey(1);

        std_msgs::Header header;
        header.stamp = ros::Time::now();
        header.frame_id = "OAK-D-right";

        sensor_msgs::ImagePtr depthmap_msg =
            cv_bridge::CvImage(
                header, sensor_msgs::image_encodings::TYPE_16UC1, depth_map)
                .toImageMsg();
        depth_map_pub_.publish(depthmap_msg);

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
