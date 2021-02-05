#include <vector>
#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>  

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
    Eigen::Vector3f point;
    std::vector<std::string> all_point;
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

    std::string csv_file = "/home/minieye/桌面/ProjImg/1381.660093";
    std::string img_file = "/home/minieye/桌面/ProjImg/1175.jpg";
    std::ifstream infile(csv_file.c_str());
    if(!infile.is_open())
    {
        std::cout << "文件读取失败" << std::endl;
        return 1;
    }

    std::string str_line;
    while(getline(infile, str_line))
    {
        std::stringstream ss(str_line);

        std::string str_tmp;
        while(getline(ss, str_tmp, ','))
            //point << atoi(str_tmp);
            all_point.push_back(str_tmp);
    };
    infile.close();

    // Matx31f Rvec = Matx31f(0.1161561848857998, -2.197102120983782, 2.167617993132904);//单位是幅度,角度是(0,90左右,0)
	// std::cout << "转换成旋转矩阵后"<<to_Rmatrix(Rvec)<< std::endl;

    // point << stringToNum<float>(all_point[0]), stringToNum<float>(all_point[1]), stringToNum<float>(all_point[2]);
    // point_cloud.push_back(point);

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
    // for (int i = 0; i< point_cloud.size(); i++)
    // {
    //     std::cout << "i" << i << std::endl<< point_cloud[i] << std::endl;
    // }

    std::vector<Eigen::Vector2f> PixelPoint = projPts2Img(point_cloud, true);
    // for (int i = 0; i< PixelPoint.size(); i++)
    // {
    //     std::cout << "i" << i << std::endl<< PixelPoint[i].transpose() << std::endl;
    // }

    cv::Mat img = imread(img_file,1);
    for (int i = 0; i < PixelPoint.size() ; i++)
    {   
        auto& pt = PixelPoint[i];
        cv::Point2f cv_pt(pt[0], pt[1]);
        // cv::eigen2cv(PixelPoint[i], cv_point);
        // std::cout << cv_pt << std::endl;
        cv::circle(img, cv_pt, 1, CV_RGB(255,0,0), 2);
    }
    imshow("test", img);
    waitKey(0);






    return 1;
}
