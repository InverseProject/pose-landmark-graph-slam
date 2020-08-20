#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>


struct IntrinsicMatrix
{
    double center_u;
    double center_v;
    double focal_u;
    double focal_v;
};

class InverseProjection
{
    public:
        InverseProjection(const IntrinsicMatrix intrinsic, const cv::Mat depthmap);
        ~InverseProjection();
        IntrinsicMatrix get_intrinsic() const;
        void set_intrinsic(const IntrinsicMatrix new_intrinsic);
        cv::Mat get_depthmap() const;
        void set_depthmap(const cv::Mat new_depthmap);
        Eigen::MatrixXd convert_cv_mat_to_eigen_mat(cv::Mat mat);
        Eigen::MatrixXd get_pointcloud_from_depthmap();
        pcl::PointCloud<pcl::PointXYZ> get_pcl_pointcloud();
        void save_pointcloud_to_pcd(std::string save_file_path);


    private:
        IntrinsicMatrix intrinsic_matrix_;
        cv::Mat depthmap_;
};

InverseProjection::InverseProjection(const IntrinsicMatrix intrinsic, const cv::Mat depthmap)
{
    intrinsic_matrix_ = intrinsic;
    depthmap_ = depthmap;
}

InverseProjection::~InverseProjection() = default;

IntrinsicMatrix InverseProjection::get_intrinsic() const
{
    return intrinsic_matrix_;
}

void InverseProjection::set_intrinsic(const IntrinsicMatrix new_intrinsic)
{
     intrinsic_matrix_ = new_intrinsic;
}

cv::Mat InverseProjection::get_depthmap() const
{
    return depthmap_;
}

void InverseProjection::set_depthmap(const cv::Mat new_depthmap)
{
    depthmap_ = new_depthmap;
}

Eigen::MatrixXd InverseProjection::convert_cv_mat_to_eigen_mat(cv::Mat mat)
{
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> mtrx;
    cv::cv2eigen(depthmap_, mtrx);
    return mtrx;
}

Eigen::MatrixXd InverseProjection::get_pointcloud_from_depthmap()
{
    auto mtrx = convert_cv_mat_to_eigen_mat(depthmap_);
    Eigen::MatrixXd camera_coordinate(mtrx.rows()*mtrx.cols(), 3);

    unsigned long int idx = 0;

    /*
    * Note: accessing every pixel with 2 for loops quite inefficient.
    * <TODO> implement in a way to perform matrix operations.
    */
    for(int v = 0; v < mtrx.rows(); v++)
    {
        for(int u = 0; u < mtrx.cols(); u++)
        {
            double z = mtrx(v,u);
            double x = (u - intrinsic_matrix_.center_u) * z / intrinsic_matrix_.focal_u;
            double y = (v - intrinsic_matrix_.center_v) * z / intrinsic_matrix_.focal_v;

            camera_coordinate.row(idx) << x, y, z;
            idx++;
        }
    }


    return camera_coordinate;
}

pcl::PointCloud<pcl::PointXYZ> InverseProjection::get_pcl_pointcloud()
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    auto cam_3d_points = get_pointcloud_from_depthmap();

    cloud.width = cam_3d_points.rows();
    cloud.height = 1;
    cloud.is_dense = true;
    cloud.points.resize(cloud.width * cloud.height);

    for(unsigned long int i = 0; i < cloud.points.size(); i++){
        cloud.points[i].x = static_cast<float>(cam_3d_points(i,0));
        cloud.points[i].y = static_cast<float>(cam_3d_points(i,1));
        cloud.points[i].z = static_cast<float>(cam_3d_points(i,2));
    }

    return cloud;
}

void InverseProjection::save_pointcloud_to_pcd(std::string save_file_path)
{
    std::string file_name = "saved_pcd.pcd";
    if(!save_file_path.empty()) save_file_path += file_name;
    
    auto cloud_to_save = get_pcl_pointcloud();
    pcl::io::savePCDFileASCII(save_file_path, cloud_to_save);
}