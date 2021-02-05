#include "InitParam/params_init.h"
#include "calib/calib.h"
#include <iostream>
#include <vector>
#include <string>


int main()
{
    /*1 参数初始化*/
    std::string configFile = "/home/minieye/桌面/calib/parameter/vls128.txt" ;
    INIT init(configFile);
    std::cout << "参数配置文件输入完成！" << std::endl;

    /*2 载入标定用图片*/
    std::vector<std::string> img_files;
    init.GetFile(init.srcImg_path, img_files);
    std::cout << "载入标定用图片:" << img_files.size() << std::endl;
    for(int i = 0; i < (int)img_files.size(); i++){ //验证文件顺序是否正确
       std::cout << img_files[i] << std::endl;
    }

    /*3 载入标定用点云数据*/
    std::vector<std::string> csv_files;
    init.GetFile(init.srclidarPoints_path, csv_files);
    std::cout << "载入标定用点云数据:" << csv_files.size() << std::endl;

     /*4 图像 点云数据匹配*/
    init.FileMatch(init.log_txt_path, img_files, csv_files);

    CALIB calib;
    /* 5 循环读取图像和点云数据，并提取对应特征 */
    int success_(0), fail_(0); //用来记录图像和激光点云对应特征提取成功的帧数
    for(int i = 0; i < (int)img_files.size(); i++)
    {
        /* 5.1读取图像文件 */
        cv::Mat img = calib.ReadImg(img_files[i], init.srcImg_path);
        /* 5.2读取csv文件 */
        pcl::PointCloud<PointXYZIR>::Ptr laserCloudIn = calib.ReadCsv(init.srclidarPoints_path, init.need_csv_files[i]) ;
        // std::cout << "读入点云数据" << std::endl;
        // calib.visualize_pcd(laserCloudIn); // 查看录入的点云数据

        /* 5.3图像特征提取 */
        std::vector<cv::Point2f> img_board_coenerAndCenter = calib.GetImgFeature(img, init.grid_size, init.square_length, init.board_dimension, init.cameramat, init.distcoeff);
        /* 5.4点云特征提取 */
        //calib.GetLidarFeature(laserCloudIn, init.x_range_point_filter, init.y_range_point_filter, init.z_range_point_filter, init.plane_line_dist_threshold, init.lidar_ring_count, init.line_fit2real_dist);

        //std::cout << calib.all_lidar_board_coenerAndCenter.size() <<std::endl;
        //std::cout << calib.all_img_board_coenerAndCenter.size() <<std::endl;
        if(!img_board_coenerAndCenter.empty())
        {

            if(calib.GetLidarFeature(laserCloudIn, init.x_range_point_filter, init.y_range_point_filter, init.z_range_point_filter, init.plane_line_dist_threshold, init.lidar_ring_count, init.line_fit2real_dist))
            {
                std::cout << img_files[i] << " 和 " << init.need_csv_files[i] << "对应特征提取成功！" << std::endl;
                success_ ++;
                calib.all_img_board_coenerAndCenter.insert(calib.all_img_board_coenerAndCenter.end(), img_board_coenerAndCenter.begin(), img_board_coenerAndCenter.end());
                calib.all_lidar_board_coenerAndCenter.insert(calib.all_lidar_board_coenerAndCenter.end(), calib.lidar_board_coenerAndCenter.begin(), calib.lidar_board_coenerAndCenter.end());
                for(int i = 0; i < calib.lidar_board_coenerAndCenter.size() ; i ++)
                {
                    std::cout << calib.lidar_board_coenerAndCenter[i] << std::endl;
                }
                std::cout << "img_board_coenerAndCenter size: " << img_board_coenerAndCenter.size() << std::endl;
                std::cout << "lidar_board_coenerAndCenter size: " << calib.lidar_board_coenerAndCenter.size() << std::endl;
                img_board_coenerAndCenter.clear();   //清空数据,等待下一组数据的录入
                calib.lidar_board_coenerAndCenter.clear(); //清空数据，等待下一组数据的录入
            }
            else
            {
                std::cout << img_files[i] << " 和 " << init.need_csv_files[i] << "对应特征提取失败！" << endl;
                fail_++;
                img_board_coenerAndCenter.clear();   //清空数据,等待下一组数据的录入
                calib.lidar_board_coenerAndCenter.clear(); //清空数据，等待下一组数据的录入
                continue;
            }
        }

    };
    std::cout << "用于对应特征提取的图像和点云组数为：" << success_ + fail_ << std::endl;
    std::cout << "图像和激光点云对应特征提取成功组数：" << success_ << std::endl;
    std::cout << "图像和激光点云对应特征提取失败组数：" << fail_ << std::endl;


    /* 6 求解camera和lidar的外参 */
    std::cout << "all_lidar_board_coenerAndCenter size:" << calib.all_lidar_board_coenerAndCenter.size() <<std::endl;
    std::cout << "all_img_board_coenerAndCenter   size " << calib.all_img_board_coenerAndCenter.size() <<std::endl;
    cv::solvePnP(calib.all_lidar_board_coenerAndCenter, calib.all_img_board_coenerAndCenter, init.cameramat, init.distcoeff, calib.R_camToLidar, calib.T_camToLidar, false, cv::SOLVEPNP_EPNP); // pnp模型求R,T
    cv::solvePnP(calib.all_lidar_board_coenerAndCenter, calib.all_img_board_coenerAndCenter, init.cameramat, init.distcoeff, calib.R_camToLidar, calib.T_camToLidar, true, cv::SOLVEPNP_ITERATIVE); // pnp模型求R,T
    cv::Mat r;
    cv::Rodrigues(calib.R_camToLidar, r); // R 为旋转向量形式，用 Rodrigues 公式转换为矩阵

    std::cout << "lidar 到 camera 的外参 R：" << std::endl << calib.R_camToLidar << std::endl;
    std::cout << "lidar 到 camera 的外参 T：" << std::endl << calib.T_camToLidar << std::endl;


   /*7 camera-lidar外参标定结果保留 */
   ofstream fout((init.calibration_result_path + "camera_lidar_calibration_result.txt").c_str(), ios::out | ios::trunc);
   fout << calib.R_camToLidar << std::endl;
   fout << calib.T_camToLidar << std::endl;


    std::cout << "calib.chessBoard_lidar_points size:" << calib.chessBoard_lidar_points.size() << std::endl;
    std::cout << "img_files size：" << img_files.size() <<std::endl;

    /*8 将标定板点云按照对应关系映射到图像上*/
    calib.Project3DPoints(calib.chessBoard_lidar_points, calib.R_camToLidar, calib.T_camToLidar, init.srcImg_path, init.cameramat, init.distcoeff, img_files, init.img_projection_path);

    return 1;
}



