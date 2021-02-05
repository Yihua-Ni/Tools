#include <string>
#include <fstream>
#include <sstream>          //isdtream
#include <iostream>
#include <string>
#include <vector>
#include <sys/types.h>   //getfile
#include <dirent.h>      //getfile
#include "params_init.h"

INIT::INIT(const std::string& filepath)
{
    std::ifstream infile(filepath.c_str());
    infile >> srcImg_path;
    infile >> srclidarPoints_path;
    infile >> log_txt_path;
    infile >> img_projection_path;
    infile >> calibration_result_path;
    infile >> lidar_ring_count;
    infile >> width_grid_count;
    infile >> height_grid_size;
    grid_size = std::make_pair(width_grid_count, height_grid_size);
    infile >> square_length; 
    infile >> board_width;
    infile >> board_height; 
    board_dimension = std::make_pair(board_width, board_height);

    for (int i = 0; i < 9; i++) {
        infile >> camera_mat[i];
    }
    cv::Mat(3, 3, CV_64F, &camera_mat).copyTo(cameramat); //相机内参
    //std::cout << cameramat << std::endl;
    for (int i = 0; i < 5; i++) {
        infile >> dist_coeff[i];
    }
    cv::Mat(1, 5, CV_64F, &dist_coeff).copyTo(distcoeff);
    //std::cout << distcoeff << std::endl;

    infile >> plane_dist_threshold;
    infile >> line_dist_threshold;
    plane_line_dist_threshold = std::make_pair(plane_dist_threshold, line_dist_threshold); //ransac平面拟合和直线拟合的距离阈值

    infile >> x_range_down;
    infile >> x_range_up;
    x_range_point_filter = std::make_pair(x_range_down, x_range_up); //点云滤波x范围：down < x <up

    infile >> y_range_down;
    infile >> y_range_up;
    y_range_point_filter = std::make_pair(y_range_down, y_range_up); //点云滤波y范围：down < y <up

    infile >> z_range_down;
    infile >> z_range_up;
    z_range_point_filter = std::make_pair(z_range_down, z_range_up); //点云滤波z范围：down < z <up

    infile >> line_fit2real_dist; //求出的标定板的四个角点构成的矩形边长与真值的差值，单位：m

    diagonal = sqrt(pow(board_dimension.first, 2) + pow(board_dimension.second, 2)) / 1000;
    std::cout << "棋盘格对角线长度为：" << diagonal << "m!" << std::endl;
    //std::cout << "参数配置文件输入完成！" << std::endl;
    return;
}



/******   文件夹文件读取      ******/
void INIT::GetFile(std::string path,std::vector<std::string>& file)
{

    char buf1[1000];
    DIR *pDir;
    struct dirent* ptr;
    if(!(pDir = opendir(path.c_str())))
        return;

    while((ptr = readdir(pDir))!=0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            file.push_back(ptr->d_name);
        }
    }

    sort(file.begin(),file.end());

    closedir(pDir);

    return;
};


/* 图像、点云数据匹配，时间最近即为匹配 */
int INIT::FileMatch(std::string log_txt_path, std::vector<std::string> img_files, std::vector<std::string> csv_files)
{
    //std::ifstream log_txt(log_txt_path.c_str(), ios::in);
    std::ifstream log_txt(log_txt_path.c_str(), std::ios::in);
    if(log_txt.fail())
    {
        std::cout << "Couldn't load the log_txt file!:" << log_txt_path << std::endl;
        return -1;
    }

    std::vector<std::vector<double> > all_img_time_id;
    std::string every_line_str("");
    while(getline(log_txt, every_line_str))
    {
        // std::cout << every_line_str << std::endl;
        std::istringstream sin(every_line_str);
        std::vector<std::string> tmp_line_str;
        std::string tmp_str("");
        while(getline(sin, tmp_str, ' '))
        {
            tmp_line_str.push_back(tmp_str);
        }

        // std::cout << tmp_line_str[2] << std::endl;
        if(tmp_line_str[2] == "camera" || tmp_line_str[2] == "cam_frame")
        {
            std::vector<double> one_img_time_id(2);
            one_img_time_id[0] = atof((tmp_line_str[0] + "." + tmp_line_str[1]).c_str());
            one_img_time_id[1] = atof(tmp_line_str[4].c_str());
            all_img_time_id.push_back(one_img_time_id);
        }
    }
    // sort(all_img_time_id.begin(), all_img_time_id.end(), my_sortFunction); //排序
    //std::cout << "all_img_time_id size: " << (int)all_img_time_id.size() << std::endl;

    std::vector<std::vector<double> > need_img_time_id;
    for(size_t i = 0; i < (int)img_files.size(); i++)
    {
        for(size_t j = 0; j < (int)all_img_time_id.size(); j++)
        {
            // std::cout << atoi(img_files[i].c_str()) << "," << (int)all_img_time_id[j][1] << std::endl;
            if(atoi(img_files[i].c_str()) == (int)all_img_time_id[j][1])            //atoi()   字符串转数字
            {
            std::vector<double> tmp(2);
            tmp[0] = all_img_time_id[j][0];
            tmp[1] = all_img_time_id[j][1];
            // need_img_time_id.push_back({all_img_time_id[j][0], all_img_time_id[j][1]});
            need_img_time_id.push_back(tmp);
            break;
            }
        }
    }
    //std::cout << "need img size: " << (int)need_img_time_id.size() << std::endl;

    //std::cout << "all_csv size: " << csv_files.size() <<std::endl;

    //std::vector<std::string> need_csv_files;
    //std::vector<std::string> need_img_files;
    for(size_t i = 0; i < (int)need_img_time_id.size(); i++)
    {
        //std::cout << "img id: " << need_img_time_id[i][1] << std::endl;
        for(size_t j = 0; j < (int)csv_files.size(); j++)
        {   
            //printf("%f", need_img_time_id[i][0]) ;
            //printf("%f", atof(csv_files[j].c_str())) ;
            //std::cout << std::endl;
            //if(fabs(need_img_time_id[i][0] - atof(csv_files[j].c_str())) <= 0.05)  //fabs绝对值    atof()str转float
            if(fabs(fmod(double(need_img_time_id[i][0]), 3600) - atof(csv_files[j].c_str())) <= 0.05)
            {
                need_csv_files.push_back(csv_files[j]);
                //need_img_files.push_back(to_string(need_img_time_id[i][0]));
                break;
            }
        }
    }
    //std::cout<<"need csv size:" << need_csv_files.size()<<std::endl;

    // sort(need_csv_files.begin(), need_csv_files.end()); //排序
    //check 顺序

    for(size_t i = 0; i < (int)need_csv_files.size(); i++)
    {
        std::cout << need_csv_files[i] << std::endl;   
    }

    if(need_csv_files.size() == need_img_time_id.size())
    {
        std::cout << "成功匹配" << need_csv_files.size() << "对img和csv文件!" << std::endl;
    }
    else
    {
        std::cout << "文件匹配失败!" << std::endl;
        return -1;
    };

    return 0;
};
