#include <iostream>
#include <vector>
#include <string>

#include <Eigen/Dense>
#include "opencv2/opencv.hpp"

#include "../lib/cnpy.h"
#include "../lib/inverse_perspective_projection.h"

cv::Mat load_depth_map(std::string file_path)
{
    cnpy::NpyArray npy_data = cnpy::npy_load("../test/000317.npy");
    int data_row = npy_data.shape[0];
    int data_col = npy_data.shape[1];

    double* ptr = static_cast<double *>(malloc(data_row * data_col * sizeof(double)));
    memcpy(ptr, npy_data.data<double>(), data_row * data_col * sizeof(double));
    cv::Mat depth_map = cv::Mat(cv::Size(data_col, data_row), CV_64F, ptr);

    return depth_map;
}

int main(int argc, char** argv)
{
    // create a mock intrinsic matrix
    Eigen::Matrix3f K;
    K << 939.4981, 0.0, 279.3346,
         0.0, 939.4981, 258.0319,
         0.0, 0.0, 1.0;

    IntrinsicMatrix mock_intrinsic {K(0,2), K(1,2), K(0,0), K(1,1)};

    // // load a mock depth map
    std::string npy_file_path = "../test/000317.npy";
    cv::Mat mock_depth_map = load_depth_map(npy_file_path);

    // create a inverse projection object
    InverseProjection mock_object(mock_intrinsic, mock_depth_map);

    

    return 0;
}