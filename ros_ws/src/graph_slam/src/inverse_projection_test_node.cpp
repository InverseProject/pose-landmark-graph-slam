#include <string>

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
// #include <opencv2/imgcodecs.hpp>

#include <graph_slam/inverse_perspective_projection.h>

cv::Mat load_depth_map(std::string file_path)
{
    cv::Mat depth_map = cv::imread(file_path, CV_LOAD_IMAGE_UNCHANGED);
    // depth_map.convertTo(depth_map, CV_32F);
    return depth_map;
}

int main(int argc, char** argv)
{
    // create a mock intrinsic matrix
    Eigen::Matrix3f mock_K;
    mock_K << 851.67697279, 0.0, 628.07270979,
            0.0, 855.55430548, 354.57494198,
            0.0, 0.0, 1.0;

    // load a mock depth map
    std::string image_path = "/home/sean/Desktop/pose-landmark-graph-slam/ros_ws/src/graph_slam/test/test_maps_293.png";
    cv::Mat mock_depth_map = load_depth_map(image_path);

    // create a inverse projection object
    graph_slam::InverseProjection mock_object(mock_K, mock_depth_map);
    
    mock_object.save_pointcloud_to_pcd("/home/sean/Desktop/pose-landmark-graph-slam/ros_ws/src/graph_slam/test/");

    return 0;
}