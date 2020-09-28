#include "ros/ros.h"
#include "std_msgs/String.h"

#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

/*
 * Simple depth map subscriber for cv visualization and testing
 * 
 */ 
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // converting image ros msg to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);   
    cv::Mat img = cv_ptr->image;
    // visualizing gray scaled depth map
    cv::imshow("direct view", img);
    cv::waitKey(1);
    // getting min and max in cv::Mat of depth map
    double min, max;
    cv::minMaxLoc(img, &min, &max);
    std::cout << min << "   " << max << std::endl;

    // inverting depth_map for color map visualization
    for (int i = 0; i < img.rows; i++)
    {
        for (int j = 0; j < img.cols; j++)
        {   
            img.at<uint16_t>(i, j) = static_cast<uint16_t>(65535 / img.at<uint16_t>(i, j));
        }
    }
    // convert 16 bit depth map to 8 bit and applyinh color map for visualization
    img.convertTo(img, CV_8UC1);
    cv::applyColorMap(img, im_color,  cv::COLORMAP_HOT );
    cv::imshow("colored view", im_color);
    cv::waitKey(1);

  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "depth_map_subscriber");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // creating a subscriber for depth map
    ros::Subscriber sub = n.subscribe("depth_map", 10, imageCallback);
    ros::spin();

    return 0;
}