#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

#include <graph_slam/depthmap_to_pointcloud_converter.h>

cv::Mat load_depthmap(std::string file_path)
{
    cv::Mat depth_map = cv::imread(file_path, CV_LOAD_IMAGE_UNCHANGED);
    return depth_map;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "depth_to_pc_test_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    std::vector<float> intrinsics;
    int subsample_factor = 1;
    std::string depthmap_path = "";
    std::string pcd_save_path = "";

    int bad_params = 0;

    bad_params += !pnh.getParam("/intrinsic_matrix", intrinsics);
    bad_params += !pnh.getParam("subsample_factor", subsample_factor);
    bad_params += !pnh.getParam("depthmap_path", depthmap_path);
    bad_params += !pnh.getParam("pcd_save_path", pcd_save_path);


    if(bad_params > 0)
    {
        std::cout << "Setting one or more parameters failed. Program exiting." << std::endl;
        return 1;
    }
    
    Eigen::Matrix3f intrinsics_eigen = Eigen::Map<Eigen::Matrix3f>(intrinsics.data());

    cv::Mat mock_depth_map = load_depthmap(depthmap_path);

    graph_slam::DepthmapToPointCloudConverter convert_depthmap_to_pc(intrinsics_eigen);
    
    convert_depthmap_to_pc.save_pointcloud_to_pcd(
        mock_depth_map,
        pcd_save_path,
        subsample_factor);

    return 0;
}
