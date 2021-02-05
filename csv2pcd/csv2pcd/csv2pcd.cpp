//#include "pch.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using namespace std;
 
typedef struct tagPOINT_3D
{
	double x;  //mm world coordinate x
	double y;  //mm world coordinate y
	double z;  //mm world coordinate z
	double i;
}POINT_WORLD;

// vector<tagPOINT_3D> my_csvPoints;
// tagPOINT_3D csvPoint;

/******   文件夹文件读取      ******/
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


int csv2pcd(string csv_path, string pcd_path)
{

	vector<tagPOINT_3D> my_csvPoints;
	tagPOINT_3D csvPoint;

	ifstream fin(csv_path);
	string line;
	int i = 0;
	while (getline(fin, line)) 
	{
		//cout << "原始字符串：" << line << endl; //整行输出
		istringstream sin(line); 
		vector<string> fields;
		string field;
		while (getline(sin, field, ','))
		{
			fields.push_back(field);
		}
		if(i!=0){
			// csvPoint.x  = atof(fields[3].c_str());
			// csvPoint.y  = atof(fields[4].c_str());
			// csvPoint.z  = atof(fields[5].c_str());
			// csvPoint.i  = atof(fields[6].c_str());
			csvPoint.x  = atof(fields[0].c_str());
			csvPoint.y  = atof(fields[1].c_str());
			csvPoint.z  = atof(fields[2].c_str());
			csvPoint.i  = atof(fields[3].c_str());
			//cout << "处理之后的字符串：" << csvPoint.x << "\t" << csvPoint.y << "\t" << csvPoint.z << "\t" << csvPoint.i << endl;
			my_csvPoints.push_back(csvPoint);
		}
		else
			i++;
	}
	//cout << my_csvPoints.size() << endl;
	int number_Txt= my_csvPoints.size();
	
	pcl::PointCloud<pcl::PointXYZI> cloud;
	// Fill in the cloud data
	cloud.width = number_Txt/128;
	cloud.height = 128;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (size_t i = 0; i < cloud.points.size(); ++i)
	{
		cloud.points[i].x = my_csvPoints[i].x;
		cloud.points[i].y = my_csvPoints[i].y;
		cloud.points[i].z = my_csvPoints[i].z;
		cloud.points[i].intensity = my_csvPoints[i].i;
	}
	pcl::io::savePCDFileASCII(pcd_path, cloud);
	std::cerr << "Saved " << cloud.points.size() << " data points to csv2pcd.pcd." << std::endl;
	
	return EXIT_SUCCESS;
};



int main()
{

	string srcImg_path = "/home/minieye/桌面/1202vls128/img_data/img/converted_data/1/";
	string srclidarPoints_path = "/home/minieye/桌面/1202vls128/csv/";
	string log_txt_path = "/home/minieye/桌面/1202vls128/img_data/img/converted_data/log.txt";
	string pcd_file_path = "/home/minieye/桌面/1202vls128/csv2pcd/pcd/";

	/* 载入标定用图片 */
	std::vector<string> img_files;
	GetFile(srcImg_path, img_files);

	 /* 载入标定用点云数据 */
	std::vector<string> csv_files;
    GetFile(srclidarPoints_path, csv_files);

	std::ifstream log_txt(log_txt_path.c_str(), ios::in);       //ios::in 为了输入打开文件   log_txt为打开的文件
	if(log_txt.fail())
	{
		std::cout << "Couldn't load the log_txt file!, " << log_txt_path << std::endl;
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
			one_img_time_id[0] = atof((tmp_line_str[0] + "." + tmp_line_str[1]).c_str());
			one_img_time_id[1] = atof(tmp_line_str[4].c_str());
			all_img_time_id.push_back(one_img_time_id);
		}
	}
	// sort(all_img_time_id.begin(), all_img_time_id.end(), my_sortFunction); //排序
	std::cout << "all_img_time_id size: " << (int)all_img_time_id.size() << std::endl;

	std::vector<std::vector<double> > need_img_time_id;
	for(size_t i = 0; i < (int)img_files.size(); i++)                                  //1从log文件取出所有图片帧号  2按img_files帧号到取出的帧号取帧号
	{
		for(size_t j = 0; j < (int)all_img_time_id.size(); j++)                                            
		{
			// std::cout << atoi(img_files[i].c_str()) << "," << (int)all_img_time_id[j][1] << std::endl;
			if(atoi(img_files[i].c_str()) == (int)all_img_time_id[j][1])
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
	std::cout << "all img size: " << (int)need_img_time_id.size() << std::endl;                  //need_img_time_id.size()=img_files.size()

	std::cout << "all csv file:" << csv_files.size() <<std::endl;

	std::vector<string> need_csv_files;
	std::vector<string> need_img_files;
	for(size_t i = 0; i < (int)need_img_time_id.size(); i++)
	{
		//std::cout << "img id: " << need_img_time_id[i][1] << std::endl;
		for(size_t j = 0; j < (int)csv_files.size(); j++)
		{
			//if(fabs(need_img_time_id[i][0] - atof(csv_files[j].c_str())) <= 0.05)
       		if(fabs(fmod(double(need_img_time_id[i][0]), 3600) - atof(csv_files[j].c_str())) <= 0.05)
			{
				need_csv_files.push_back(csv_files[j]);
				need_img_files.push_back(to_string(int(need_img_time_id[i][1])));
				break;
			}

		}
	}

 	std::cout<<"need csv size:" << need_csv_files.size()<<endl;
	std::cout<<"need img size:" << need_img_files.size()<<endl;
	for (int i = 0; i < (int)need_img_files.size(); i++)
	{
		std::cout << "need_img_file_i:" << need_img_files[i] << std::endl;
	}

 	for(int i = 0; i < (int)need_csv_files.size(); i++)
 	{ 
 	  string csv_file_path = srclidarPoints_path + need_csv_files[i];
 	  std::cout << csv_file_path << std::endl;
 	  string pcd_file_path_i = pcd_file_path + to_string(int(need_img_time_id[i][1])) + ".pcd";
 	  std::cout << pcd_file_path_i << std::endl;
 	  csv2pcd(csv_file_path, pcd_file_path_i);

    }


};





