#include "ros/ros.h"

#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <limits>

/*
 * This subscriber is just for testing/debugging purposes
 */

/*
 * Simple depth map subscriber for cv visualization and testing.
 * @param msg (sensor_msgs::ImageConstPtr) : receives the depth map
 *  from the publisher in the image format.
 */
void DepthMapCallback(const sensor_msgs::ImageConstPtr& msg)
{

    // converting image ros msg to cv::Mat
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat img = cv_ptr->image;
    // visualizing gray scaled depth map
    cv::imwrite("test_image.bmp", img);

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
            img.at<uint16_t>(i, j) = static_cast<uint16_t>(
                std::numeric_limits<uint16_t>::max() / img.at<uint16_t>(i, j));
        }
    }
    // convert 16 bit depth map to 8 bit and applying color map for visualization
    img.convertTo(img, CV_8UC1);
    cv::Mat im_color;
    cv::applyColorMap(img, im_color, cv::COLORMAP_HOT);
    cv::imshow("colored view", im_color);

    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_map_subscriber_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    // creating a subscriber for depth map
    ros::Subscriber sub = n.subscribe("depth_map", 10, DepthMapCallback);
    ros::spin();

    return 0;
}
