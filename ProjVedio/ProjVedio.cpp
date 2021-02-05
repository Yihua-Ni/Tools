#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>  

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

template <class Type> 
Type stringToNum(const string& str){ 
    istringstream iss(str); 
    Type num; 
    iss >> num; 
    return num;     
} 

Matx33f to_Rmatrix(Matx31f Rvec)
{
	Matx33f dst;
	cv::Rodrigues(Rvec,dst,noArray());
	return dst;
}

Matx31f to_Rvec(Matx33f Rmatrix){
	Matx31f dst;
	cv::Rodrigues(Rmatrix,dst,noArray());
	return dst;
}

void GetFile(std::string path,std::vector<std::string>& file)
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
};


std::vector<Eigen::Vector2f> projPts2Img(const std::vector<Eigen::Vector3f>& pts, bool do_dist)
{
    std::vector<Eigen::Vector2f> uvs;
    float fx = 1473.5;
    float cx = 659.3324;
    float fy = 1485.6;
    float cy = 525.8925;

    for(const auto& it : pts)
    {
        if (0 > it(2))
        {
            continue;
        }
        
        Eigen::Vector3f uv = it / it(2);
        float x = uv(0);
        float y = uv(1);

        if(do_dist)
        {   
            float k1 = -0.4559;
            float k2 = 0.1212;
            float p1 = 0.0029;
            float p2 = 0.00052151;
            float k3 = 0.0;

            float xy = x * y;
            float x2 = x*x;
            float y2 = y*y;
            float r2 = x2 + y2;
            float r4 = r2 * r2;
            float r6 = r2 * r4;

            float c0 = 1 + k1 * r2 + k2 * r4 + k3 * r6;      //k1 k2 k3向畸变径
            x = c0 * x + 2 * p1 * xy + p2 * (r2 + 2 * x2);   //p1 p3 切向畸变
            y = c0 * y + 2 * p2 * xy + p1 * (r2 + 2 * y2);
        }

        uvs.emplace_back(x * fx + cx, y * fy + cy);          //像素坐标
    }

    return uvs;
};


int main()
{   
    string video_path = "/home/minieye/桌面/cityroad/video_data/20201202124619/converted_data/output.mp4";
    string srclidarPoints_path = "/home/minieye/桌面/cityroad/csv/";
	string log_txt_path = "/home/minieye/桌面/cityroad/video_data/20201202124619/converted_data/log.txt";

    Eigen::Vector3f point;
    std::vector<Eigen::Vector3f> point_cloud;
    Eigen::Matrix3f R;
    Eigen::Vector3f T;

    R << -0.99576831, -0.090657517, 0.015056624,
         -0.016279992, 0.012770027, -0.9997859,
          0.090445839, -0.99580026, -0.014191892;
    
    T << -89.0860862631191, -603.9316978646775, -788.4576768242996;
    T *= 1e-3;
    std::cout << "R" << R << std::endl;
    std::cout << "T" << T << std::endl;

    std::vector<string> csv_files;
    GetFile(srclidarPoints_path, csv_files);

    std::ifstream log_txt(log_txt_path.c_str(), ios::in);       //ios::in 为了输入打开文件   log_txt为打开的文件
    if(log_txt.fail())
    {
        std::cout << "log文件读取失败 " << log_txt_path << std::endl;
        return -1;
    }

    std::vector<std::vector<double> > all_img_time_id;
	string every_line_str("");
	while(getline(log_txt, every_line_str))
	{
		// std::cout << every_line_str << std::endl;
		istringstream sin(every_line_str);
		std::vector<std::string> tmp_line_str;
		string tmp_str("");
		while(getline(sin, tmp_str, ' '))
		{
			tmp_line_str.push_back(tmp_str);
		}

		// std::cout << tmp_line_str[2] << std::endl;
		if(tmp_line_str[2] == "camera" || tmp_line_str[2] == "cam_frame")
		{
			std::vector<double> one_img_time_id(2);
			one_img_time_id[0] = atof((tmp_line_str[0] + "." + tmp_line_str[1]).c_str()) + 0.15;
			one_img_time_id[1] = atof(tmp_line_str[4].c_str()) ;
			all_img_time_id.push_back(one_img_time_id);
		}
	}
    // for(int i = 0; i < all_img_time_id.size(); i++)
    // {
    //     cout.setf(ios::fixed,ios::floatfield);//十进制计数法，不是科学计数法
    //     std::cout <<  all_img_time_id[i][0]<< std::endl;
    // }

    VideoCapture capture1;
    Mat frame1;      // 定义一个Mat变量,用于存储每一帧图像
    frame1 = capture1.open(video_path);
    if(!capture1.isOpened())
    {
        std::cout << "video文件读取失败" << std::endl;
        return -1;
    }
    //namedWindow("output" , CV_WINDOW_AUTOSIZE);

    int time = 0;
    vector<string> csv_name;
    int flag = 0 ;

    while (capture1.read(frame1))
    {
        std::cout << "time:" << time << std::endl;

        for(size_t i = 0; i < (int)all_img_time_id.size(); i++)                                            
		{
			// std::cout << atoi(img_files[i].c_str()) << "," << (int)all_img_time_id[j][1] << std::endl;
			if(time == (int)all_img_time_id[i][1])
			{
                cout.setf(ios::fixed,ios::floatfield);//十进制计数法，不是科学计数法
                std::cout << all_img_time_id[i][0] << std::endl;
                for( int j = 0; j < csv_files.size(); j++)
                {
                    if(fabs(fmod(double(all_img_time_id[i][0]), 3600) - atof(csv_files[j].c_str())) <= 0.05)
                    {
                        std::cout << "csv_file:" << csv_files[j] << std::endl;
                        csv_name.push_back(csv_files[j]);
                        flag = 1;
                        break;
                    }


                }
                if(flag == 0)
                {
                    csv_name.push_back("");
                }
                flag = 0;

				break;
			}
		}
        time = time + 1;

    }
    std::cout << "csv  size" << csv_name.size() << std::endl;
    capture1.release();


    VideoCapture capture;
    Mat frame;
    frame = capture.open(video_path);
    if(!capture.isOpened())
    {
        std::cout << "video文件读取失败" << std::endl;
        return -1;
    }
    namedWindow("output" , CV_WINDOW_AUTOSIZE);

    int ttime = 0;
    while (capture.read(frame))
    {
        std::cout << "time:" << ttime << std::endl;

        if(csv_name[ttime] == "")
        {   
            ttime = ttime + 1;
            continue;
        }

        string csv_file_path = srclidarPoints_path + csv_name[ttime];
        //std::cout << csv_file_path << std::endl;
        std::ifstream infile(csv_file_path.c_str());
        if(!infile.is_open())
        {
            std::cout << "csv文件读取失败" << std::endl;
            return 1;
        }

        std::vector<std::string> all_point;
        std::string str_line;
        while(getline(infile, str_line))
        {
            std::stringstream ss(str_line);

            std::string str_tmp;
            while(getline(ss, str_tmp, ','))
                //point << atoi(str_tmp);
                all_point.push_back(str_tmp);
        };
        ttime = ttime + 1;
        infile.close();


        for (int i = 0; i < all_point.size(); i++)
        {
            //cout << stringToNum<float>(all_point[i]) << endl;
            int row = 0;
            if(i%5 == 0)
            {
                point << stringToNum<float>(all_point[i]), stringToNum<float>(all_point[i+1]), stringToNum<float>(all_point[i+2]);
                Eigen::Vector3f Pw;
                Pw = (R * point + T);
                //std::cout << Pw.transpose() << std::endl;
                Pw /= Pw(2);
                //cout << Pw << endl;
                point_cloud.push_back(Pw);
                row = row + 1;
            }

        }
        all_point.clear();
        // for (int i = 0; i< point_cloud.size(); i++)
        // {
        //     std::cout << "i" << i << std::endl<< point_cloud[i] << std::endl;
        // }

        std::vector<Eigen::Vector2f> PixelPoint = projPts2Img(point_cloud, true);
        // for (int i = 0; i< PixelPoint.size(); i++)
        // {
        //     std::cout << "i" << i << std::endl<< PixelPoint[i].transpose() << std::endl;
        // }
        //cv::Mat img = imread(img_file,1);
        for (int i = 0; i < PixelPoint.size() ; i++)
        {   
            auto& pt = PixelPoint[i];
            cv::Point2f cv_pt(pt[0], pt[1]);
            // cv::eigen2cv(PixelPoint[i], cv_point);
            // std::cout << cv_pt << std::endl;
            cv::circle(frame, cv_pt, 1, CV_RGB(255,0,0), 2);   //第三个参数圆半径   第五个参数线宽度
        }
        point_cloud.clear();
        PixelPoint.clear();
        

        imshow("output", frame);
        waitKey(10);
    }
    capture.release();



    return 1;
}
