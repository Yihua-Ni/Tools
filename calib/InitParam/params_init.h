#ifndef PARAMS_INIT_H
#define PARAMS_INIT_H

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class INIT
{
    public:
        std::string srcImg_path; //标定用图片的相对路径
        std::string srclidarPoints_path; //标定用点云数据的相对路径
        std::string log_txt_path; //图像log文件的相对路径

        std::pair<int, int> grid_size; //棋盘格的角点数
        int square_length; // 棋盘格每个小格子的实际尺寸,单位：mm
        std::pair<int, int> board_dimension; // 整个标定板的实际尺寸(宽，高)，单位：mm
        cv::Mat cameramat; //相机内参
        cv::Mat distcoeff; //相机的畸变系数
        std::string img_projection_path; //投影图像的保存路径

        std::pair<float, float> x_range_point_filter; //点云滤波x值范围
        std::pair<float, float> y_range_point_filter; //点云滤波y值范围
        std::pair<float, float> z_range_point_filter; //点云滤波z值范围
        std::pair<float, float> plane_line_dist_threshold; //ransac平面拟合和直线拟合的距离阈值
        int lidar_ring_count; //lidar的线数
        float line_fit2real_dist; //求出的标定板的四个角点构成的矩形边长与真值的差值阈值，单位：m

        std::vector<std::string> need_csv_files;

        std::string calibration_result_path; //标定结果保存文件的相对路径

        // int  params_init(std::string filepath);
        // INIT() = delete;
        //INIT{};
        INIT(const std::string& path);
        void GetFile(std::string path, std::vector<std::string>& file);
        int  FileMatch(std::string log_txt_path, std::vector<std::string> img_files, std::vector<std::string> csv_files);

    private:

        int width_grid_count, height_grid_size;//棋盘格角点数目(宽和高方向)
        int board_width, board_height;//棋盘格实际尺寸(宽和高方向)
        double camera_mat[9];
        double dist_coeff[5];
        double plane_dist_threshold, line_dist_threshold;
        double x_range_down, x_range_up;
        double y_range_down, y_range_up;
        double z_range_down, z_range_up;
        double diagonal;

};




#endif 

