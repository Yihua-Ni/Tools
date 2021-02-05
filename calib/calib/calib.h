#ifndef CALIBRATION_H
#define CALIBRATION_H

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>     //laserCloudIn
#include <pcl/filters/conditional_removal.h> //条件滤波器头文件
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/intersections.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/visualization/pcl_visualizer.h> //点云可视化头文件
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>	//getMinMax3D()函数所在头文件

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <deque> 
#include <math.h>
#include <Eigen/Dense>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>     //Mat
#include <opencv/cv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>     //imread imshow


struct PointXYZIR
{
  PCL_ADD_POINT4D;                    
  float    intensity;                 
  uint16_t ring;                      
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  (uint16_t, ring, ring))


// typedef struct{
//     cv::Mat R_camToLidar;
//     cv::Mat T_camToLidar;
// } RT;


class CALIB
{
    public:


        //CALIB() = delete;

        //CALIB(std::vector<cv::Point2f> all_img_board_coenerAndCenter,std::vector<cv::Point3f> all_lidar_board_coenerAndCenter, std::vector<cv::Point3f> lidar_board_coenerAndCenter, 
         //     cv::Mat R_camToLidar, cv::Mat T_camToLidar, std::vector<std::vector<cv::Point3f> > chessBoard_lidar_points, std::vector<cv::Point2f> img_board_coenerAndCenter);

        CALIB();

        std::vector<cv::Point2f> all_img_board_coenerAndCenter; // (所有帧)标定板四个角点和中心点的图像坐标,单位：像素
        std::vector<cv::Point3f> all_lidar_board_coenerAndCenter; // (所有帧)标定板四个角点和中心点在激光坐标系下的坐标，单位：mm

        std::vector<cv::Point3f> lidar_board_coenerAndCenter; // (对应帧)标定板四个角点和中心点在激光坐标系下的坐标，单位：mm

        cv::Mat R_camToLidar; //激光雷达坐标系到相机坐标系的旋转向量R,roll、pitch、yaw
        cv::Mat T_camToLidar; //激光雷达坐标系到相机坐标系的平行向量T，单位mm

        std::vector<std::vector<cv::Point3f> > chessBoard_lidar_points; // 用于存放标定板点云的vector，单位:mm

        cv::Mat ReadImg(std::string img_files, std::string srcImg_path);

        pcl::PointCloud<PointXYZIR>::Ptr ReadCsv(std::string srclidarPoints_path, std::string need_csv_files) ;

        int extractROI(const cv::Mat img, const pcl::PointCloud<PointXYZIR>::Ptr cloud, std::pair<int, int> grid_size, int square_length, std::pair<int, int> board_dimension, cv::Mat cameramat, cv::Mat distcoeff) ;

        //int GetImgFeature(cv::Mat &img, std::pair<int, int> grid_size, int square_length, std::pair<int, int> board_dimension, cv::Mat cameramat, cv::Mat distcoeff);
        std::vector<cv::Point2f> GetImgFeature(cv::Mat &img, std::pair<int, int> grid_size, int square_length, std::pair<int, int> board_dimension, cv::Mat cameramat, cv::Mat distcoeff);

        int GetLidarFeature(const pcl::PointCloud<PointXYZIR>::Ptr &cloud, std::pair<float, float> x_range_point_filter, std::pair<float, float> y_range_point_filter, std::pair<float, float> z_range_point_filter, std::pair<float, float> plane_line_dist_threshold, int lidar_ring_count, float line_fit2real_dist);

        //RT CALIB::calculate_camToLidar_RT(std::vector<cv::Point3f> all_lidar_board_coenerAndCenter, std::vector<cv::Point2f> all_img_board_coenerAndCenter, cv::Mat cameramat, cv::Mat distcoeff, cv::Mat R_camToLidar, cv::Mat T_camToLidar);

        int Project3DPoints(std::vector<std::vector<cv::Point3f> > chessBoard_lidar_points, cv::Mat R_camToLidar, cv::Mat T_camToLidar, std::string srcImg_path, cv::Mat cameramat, cv::Mat distcoeff, std::vector<std::string> img_files, std::string img_projection_path);

        void visualize_pcd(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, int red , int green , int blue);

   

   private:

        std::vector<cv::Point2f> img_board_coenerAndCenter; // (对应帧)标定板四个角点和中心点的图像坐标,单位：像素

        // cv::Mat R_camToLidar = (Mat_<double>(3, 1) << 1.25276082345453, -1.193519093120378, 1.210215014689938); //激光雷达坐标系到相机坐标系的旋转向量R,roll、pitch、yaw
        // cv::Mat T_camToLidar = (Mat_<double>(3, 1) << -72.5646427842064, -698.275374750283, -722.5815488873395); //激光雷达坐标系到相机坐标系的平行向量T，单位mm

        
        void visualize_pcd2(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, pcl::PointCloud<PointXYZIR>::Ptr &pcd_src2);
        void visualize_pcd3(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, pcl::PointCloud<PointXYZIR>::Ptr &pcd_src2, pcl::PointCloud<PointXYZIR>::Ptr &pcd_src3);
        void visualize_box(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, PointXYZIR left_up, PointXYZIR left_down, PointXYZIR right_up, PointXYZIR right_down);

        //void visualize_pcd(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src);
        // void aabb(pcl::PointCloud<PointXYZIR>::Ptr cloud);//点云AABB包围盒
        // void obb(pcl::PointCloud<PointXYZIR>::Ptr cloud);


      



};




#endif