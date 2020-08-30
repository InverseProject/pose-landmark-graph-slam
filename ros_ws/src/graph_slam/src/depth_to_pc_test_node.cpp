#include <string>
#include <opencv2/opencv.hpp>

#include <graph_slam/depthmap_to_pointcloud_converter.h>

cv::Mat load_depth_map(std::string file_path)
{
    cv::Mat depth_map = cv::imread(file_path, CV_LOAD_IMAGE_UNCHANGED);
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
    graph_slam::DepthmapToPointCloudConverter mock_object(mock_K);
    
    mock_object.save_pointcloud_to_pcd(mock_depth_map, "/home/sean/Desktop/pose-landmark-graph-slam/ros_ws/src/graph_slam/test/", 1);

    return 0;
}
