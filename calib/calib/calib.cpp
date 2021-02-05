#include "calib.h"


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>     //Mat
#include <opencv/cv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>     //imread imshow

#define pi 3.14159265

       
CALIB::CALIB()
{
   return;
};


cv::Mat CALIB::ReadImg(std::string img_files, std::string srcImg_path)
{
    std::cout << "正在读取 "<< img_files << " 文件" << std::endl; //正在处理的图像
    cv::Mat img = cv::imread(srcImg_path + img_files, 1);
    if(img.empty()){
        std::cout << "Couldn't load the image!" << std::endl; //读取图像失败
        //return -1;
    }
    return img;
};

pcl::PointCloud<PointXYZIR>::Ptr CALIB::ReadCsv(std::string srclidarPoints_path, std::string need_csv_files) 
{
    //std::ifstream csv_file((srclidarPoints_path + need_csv_files).c_str(), ios::in);
    std::cout << "正在读取 " << need_csv_files << " 文件" << std::endl; //正在处理的点云csv文件
    pcl::PointCloud<PointXYZIR>::Ptr laserCloudIn(new pcl::PointCloud<PointXYZIR>);    
    std::ifstream csv_file((srclidarPoints_path + need_csv_files).c_str());
    if(csv_file.fail()) {
        std::cout << "Couldn't load the csv !" << std::endl; //读取点云csv文件失败
        //return -1;
    }
    std::string line("");       //string line = ""   初始化空字符
    // int n = 0; //用于剔除csv的第1行而设置的变量
    while(getline(csv_file, line)){
        // cout << "csv原始字符串为：" << line << endl; // 用来校验
        std::istringstream sin(line); //istringstream类用于执行C++风格的串流的输入操作
        std::vector<std::string> parameters;
        std::string parameter;
        while (getline(sin, parameter, ','))
        {
            parameters.push_back(parameter);
        };

        PointXYZIR lidar_point;
        lidar_point.x = atof(parameters[0].c_str());
        lidar_point.y = atof(parameters[1].c_str());
        lidar_point.z = atof(parameters[2].c_str());
        lidar_point.intensity = atof(parameters[3].c_str());
        lidar_point.ring = atof(parameters[4].c_str());
        laserCloudIn->points.push_back(lidar_point);
    };

    return laserCloudIn;
};

/******   点云可视化1   ******/
void CALIB::visualize_pcd(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, int red , int green , int blue)      //0, 255, 0 绿色
{

   pcl::visualization::PCLVisualizer viewer("registration Viewer");
   pcl::visualization::PointCloudColorHandlerCustom<PointXYZIR> src_h (pcd_src, red, green, blue);
   viewer.addPointCloud (pcd_src, src_h, "source cloud");    //绿色

   while (!viewer.wasStopped())
   {
       viewer.spinOnce(100);
      //  boost::this_thread::sleep(boost::posix_time::microseconds(100000));
       //viewer.spinOnce();
   }
   viewer.close();
   return;
};

/******   点云可视化2   ******/
void CALIB::visualize_pcd2(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, pcl::PointCloud<PointXYZIR>::Ptr &pcd_src2)
{

   pcl::visualization::PCLVisualizer viewer("registration Viewer");
   pcl::visualization::PointCloudColorHandlerCustom<PointXYZIR> src_h (pcd_src, 0, 255, 0);
   pcl::visualization::PointCloudColorHandlerCustom<PointXYZIR> src_h2 (pcd_src2, 255, 0, 0);
   viewer.addPointCloud (pcd_src, src_h, "source cloud");
   viewer.addPointCloud (pcd_src2, src_h2, "source cloud2");

   while (!viewer.wasStopped())
   {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      //viewer.spinOnce();
   }
   viewer.close();
   return;
};

/******   点云可视化3   ******/
void CALIB::visualize_pcd3(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, pcl::PointCloud<PointXYZIR>::Ptr &pcd_src2, pcl::PointCloud<PointXYZIR>::Ptr &pcd_src3)
{

   pcl::visualization::PCLVisualizer viewer("registration Viewer");
   pcl::visualization::PointCloudColorHandlerCustom<PointXYZIR> src_h (pcd_src, 0, 255, 0);
   pcl::visualization::PointCloudColorHandlerCustom<PointXYZIR> src_h2 (pcd_src2, 255, 0, 0);
   pcl::visualization::PointCloudColorHandlerCustom<PointXYZIR> src_h3 (pcd_src3, 0, 0, 255);
   viewer.addPointCloud (pcd_src, src_h, "source cloud");
   viewer.addPointCloud (pcd_src2, src_h2, "source cloud2");
   viewer.addPointCloud (pcd_src3, src_h3, "source cloud3");

   while (!viewer.wasStopped())
   {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      //viewer.spinOnce();
   }
   viewer.close();
   return;
};

/******   点云可视化box   ******/
void CALIB::visualize_box(pcl::PointCloud<PointXYZIR>::Ptr &pcd_src, PointXYZIR left_up, PointXYZIR left_down, PointXYZIR right_up, PointXYZIR right_down)
{
   pcl::visualization::PCLVisualizer viewer("registration Viewer");
   pcl::visualization::PointCloudColorHandlerCustom<PointXYZIR> src_h (pcd_src, 0, 255, 0);

   viewer.addPointCloud (pcd_src, src_h, "source cloud");
   viewer.addLine (left_up, left_down, 255, 0, 0, "left");
   viewer.addLine (left_up, right_up,  255, 0, 0, "up");
   viewer.addLine (right_up, right_down,  255, 0, 0, "right");
   viewer.addLine (left_down, right_down,  255, 0, 0, "down");

   while (!viewer.wasStopped())
   {
      viewer.spinOnce(100);
      boost::this_thread::sleep(boost::posix_time::microseconds(100000));
      //viewer.spinOnce();
   }
   viewer.close();
   return;
};



std::vector<cv::Point2f> CALIB::GetImgFeature(cv::Mat &img, std::pair<int, int> grid_size, int square_length, std::pair<int, int> board_dimension, cv::Mat cameramat, cv::Mat distcoeff)
{
   cv::Mat chessboard_normal = cv::Mat(1, 3, CV_64F);              //标定板

   cv::Size2i patternNum(grid_size.first, grid_size.second);
   cv::Size2i patternSize(square_length, square_length);

   cv::Mat gray;
   std::vector<cv::Point2f> corners; // 检测到的棋盘格角点
   std::vector<cv::Point3f> grid3dpoint; // 棋盘格角点世界坐标系下的3D点,单位：mm
   cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
   // std::cout << "img cols: " << gray.cols << ", " << "img rows: " << gray.rows << "." << endl;
   // 寻找棋盘格角点
   bool patternfound = cv::findChessboardCorners(gray, patternNum, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

   if (patternfound) {
      // 寻找亚像素精度的角点
      std::cout << "成功找到角点！" << std::endl;
      cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(2 + 1, 30, 0.1));
      // 角点绘制
      cv::drawChessboardCorners(img, patternNum, corners, patternfound);

      double tx, ty; // 世界坐标系原点
      // 角点中心为世界坐标系原点
      tx = (patternNum.width - 1) * patternSize.width / 2;
      ty = (patternNum.height - 1) * patternSize.height / 2;
      // 求各个角点的世界坐标,单位：mm
      for (int i = 0; i < patternNum.height; i++) {
         for (int j = 0; j < patternNum.width; j++) {
            cv::Point3f tmpgrid3dpoint;
            tmpgrid3dpoint.x = j * patternSize.width - tx;
            tmpgrid3dpoint.y = i * patternSize.height - ty;
            tmpgrid3dpoint.z = 0;
            grid3dpoint.push_back(tmpgrid3dpoint);
         }
      }
      
      // 标定板四个边角点世界坐标(基于位置错放补偿),角点需要与标定板的激光点云前后左右中5个角点对应
      // 寻找检测的角点的中最上方、最下方、最左方、最右方四个角点，根据四个角点的检测的顺序，决定标定板上下左右中五个特征点的三维坐标，以此来和标定板的激光点云前后左右中5个角点对应
      std::vector<cv::Point3f> boardcorners;
      double up_row = 99999.0, down_row = 0.0, left_col = 99999.0, right_col = 0.0;
      int up_index, down_index, left_index, right_index;
      std::vector<int> index_sort; 
      for(int i = 0; i < (int)corners.size(); i++){
         if(corners[i].y < up_row){
            up_row = corners[i].y;
            up_index = i;
         }
         if(corners[i].y > down_row){
            down_row = corners[i].y;
            down_index = i;
         }
         if(corners[i].x < left_col){
            left_col = corners[i].x;
            left_index = i;
         }
         if(corners[i].x > right_col){
            right_col = corners[i].x;
            right_index = i;
         }
      }
      index_sort.push_back(up_index);
      index_sort.push_back(down_index);
      index_sort.push_back(left_index);
      index_sort.push_back(right_index);
      sort(index_sort.begin(), index_sort.end());

      if(index_sort[0] == up_index && index_sort[1] == left_index) { //最上点为角点检测起点，方向向左
         // 左右下上
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
      }
      else if(index_sort[0] == up_index && index_sort[1] == right_index) { //最上点为角点检测起点，方向向右
         // 左右下上
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
      }
      else if(index_sort[0] == down_index && index_sort[1] == left_index) { //最下点为角点检测起点，方向向左
         // 左右下上
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
      }
      else if(index_sort[0] == down_index && index_sort[1] == right_index) { //最下点为角点检测起点，方向向右
         // 左右下上
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
      }
      else if(index_sort[0] == right_index && index_sort[1] == up_index) { //最右点为角点检测起点，方向向上
         // 左右下上
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
      }
      else if(index_sort[0] == right_index && index_sort[1] == down_index) { //最右点为角点检测起点，方向向下
         // 左右下上
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
      }
      else if(index_sort[0] == left_index && index_sort[1] == up_index) { //最左点为角点检测起点，方向向上
         // 左右下上
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
      }
      else if(index_sort[0] == left_index && index_sort[1] == down_index) { //最左点为角点检测起点，方向向下
         // 左右下上
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(board_dimension.first / 2, -board_dimension.second / 2, 0.0));
         boardcorners.push_back(cv::Point3f(-board_dimension.first / 2, board_dimension.second / 2, 0.0));
      }
      // 标定板中心点的世界坐标(基于位置错放补偿)
      boardcorners.push_back(cv::Point3f(0.0, 0.0, 0.0)); //中心点

      cv::Mat rvec(3, 3, cv::DataType<double>::type); // 旋转向量r，世界坐标系到相机坐标系,roll、pitch、yaw
      cv::Mat tvec(3, 1, cv::DataType<double>::type); // 平移向量t，世界坐标系到相机坐标系,单位：mm

      // 普通相机模型
      cv::solvePnP(grid3dpoint, corners, cameramat, distcoeff, rvec, tvec); // pnp模型求r,t
      cv::projectPoints(boardcorners, rvec, tvec, cameramat, distcoeff, img_board_coenerAndCenter);
      // std::cout << img_board_coenerAndCenter << endl; //输出4个边角点和中心点的图像坐标,单位：像素
      

      // chessboardpose 是一个 3*4 转换矩阵，用来将标定板四个边角点和中心点的世界坐标转化为相机坐标 | R&T
      cv::Mat chessboardpose = cv::Mat::eye(4, 4, CV_64F);
      cv::Mat tmprmat = cv::Mat(3, 3, CV_64F); // 旋转矩阵
      cv::Rodrigues(rvec, tmprmat); // 将旋转向量(欧拉角)转化为旋转矩阵

      for (int j = 0; j < 3; j++) {
         for (int k = 0; k < 3; k++) {
            chessboardpose.at<double>(j, k) = tmprmat.at<double>(j, k);
         }
         chessboardpose.at<double>(j, 3) = tvec.at<double>(j);
      }
      //相机坐标系中标定板的法向量(朝向向量)
      chessboard_normal.at<double>(0) = 0;
      chessboard_normal.at<double>(1) = 0;
      chessboard_normal.at<double>(2) = 1;
      chessboard_normal = chessboard_normal * chessboardpose(cv::Rect(0, 0, 3, 3)).t();

      for (int k = 0; k < boardcorners.size(); k++) {
         if (k == 0)
            cv::circle(img, img_board_coenerAndCenter[0], 8, CV_RGB(0, 255, 0), -1); //green //左边特征点
         else if (k == 1)
            cv::circle(img, img_board_coenerAndCenter[1], 8, CV_RGB(255, 255, 0), -1); //yellow //右边特征点
         else if (k == 2)
            cv::circle(img, img_board_coenerAndCenter[2], 8, CV_RGB(0, 0, 255), -1); //blue //下边特征点
         else if (k == 3)
            cv::circle(img, img_board_coenerAndCenter[3], 8, CV_RGB(255, 0, 0), -1); //red //上边特征点
         else
            cv::circle(img, img_board_coenerAndCenter[4], 8, CV_RGB(255, 255, 255), -1); //white for centre //中心特征点
      }
      cv::imshow("img",img);
      //cv::waitKey(0);    //无限制等待按下键触发
      cv::waitKey(1000);    //无限制等待按下键触发
      //while(1){if(cv::waitKey(5)==27) break;}    //等待5毫秒 ESC 退出
      cv::destroyAllWindows(); //关闭所有可视化窗口
   } // if (patternfound)
   else{
      std::cout << "图像角点检测失败！" << std::endl;
      std::vector<cv::Point3f> tmp;
      chessBoard_lidar_points.push_back(tmp);
      return std::vector<cv::Point2f>();         
      };
    
    return img_board_coenerAndCenter;

};


int CALIB::GetLidarFeature(const pcl::PointCloud<PointXYZIR>::Ptr &cloud, std::pair<float, float> x_range_point_filter, std::pair<float, float> y_range_point_filter, 
                                                std::pair<float, float> z_range_point_filter, std::pair<float, float> plane_line_dist_threshold, int lidar_ring_count, float line_fit2real_dist)
{
 //////////////// 点云特征提取 //////////////////

   // 点云滤波
   pcl::PointCloud<PointXYZIR>::Ptr cloud_passthrough(new pcl::PointCloud<PointXYZIR>); // 第一次滤波后点云
   // 第一次点云滤波
   pcl::ConditionAnd<PointXYZIR>::Ptr range_condition(new pcl::ConditionAnd<PointXYZIR>()); // 点云过滤器
   // x轴过滤
   range_condition->addComparison(pcl::FieldComparison<PointXYZIR>::ConstPtr(new
   pcl::FieldComparison<PointXYZIR>("x", pcl::ComparisonOps::GT, x_range_point_filter.first)));  // GT表示大于等于
   range_condition->addComparison(pcl::FieldComparison<PointXYZIR>::ConstPtr(new
   pcl::FieldComparison<PointXYZIR>("x", pcl::ComparisonOps::LT, x_range_point_filter.second)));  // LT表示小于等于
   // y轴过滤
   range_condition->addComparison(pcl::FieldComparison<PointXYZIR>::ConstPtr(new
   pcl::FieldComparison<PointXYZIR>("y", pcl::ComparisonOps::GT, y_range_point_filter.first)));  // GT表示大于等于
   range_condition->addComparison(pcl::FieldComparison<PointXYZIR>::ConstPtr(new
   pcl::FieldComparison<PointXYZIR>("y", pcl::ComparisonOps::LT, y_range_point_filter.second)));  // LT表示小于等于
   // z轴过滤
   range_condition->addComparison(pcl::FieldComparison<PointXYZIR>::ConstPtr(new
   pcl::FieldComparison<PointXYZIR>("z", pcl::ComparisonOps::GT, z_range_point_filter.first)));  // GT表示大于等于
   range_condition->addComparison(pcl::FieldComparison<PointXYZIR>::ConstPtr(new
   pcl::FieldComparison<PointXYZIR>("z", pcl::ComparisonOps::LT, z_range_point_filter.second)));  // LT表示小于等于
 
   pcl::ConditionalRemoval<PointXYZIR> condition;
   condition.setCondition(range_condition);
   condition.setInputCloud(cloud); // 输入点云
   condition.setKeepOrganized(false);
   condition.filter(*cloud_passthrough);
   std::cout << "滤波前点云数：" << cloud->points.size() << std::endl;
   std::cout << "第一次滤波后点云数：" << cloud_passthrough->points.size() << std::endl;
   // 第一次滤波后点云可视化
   //visualize_pcd(cloud_passthrough, 0, 255, 0);    //绿色

/***********************************去除地面点**********************************************/
   pcl::PointCloud<PointXYZIR>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIR>);
   // Filter out the board point cloud
   // find the point with max height(z val) in cloud_passthrough
   PointXYZIR cloud_min, cloud_max;
   pcl::getMinMax3D(*cloud_passthrough, cloud_min, cloud_max);
   double z_max = cloud_max.z;
   // subtract by approximate diagonal length (in metres)
   double diag = hypot(1000.0, 1000.0) /                      //标定板高度 标定板宽度
                  1000.0;  // board dimensions are in mm
   double z_min = z_max - diag;
   pcl::PassThrough<PointXYZIR> pass_z;
   pass_z.setFilterFieldName("z");
   pass_z.setFilterLimits(z_min, z_max);
   pass_z.setInputCloud(cloud_passthrough);
   pass_z.filter(*cloud_filtered);  // board point cloud
   std::cout << "去除地面点云后点云数: " << cloud_filtered->points.size() << std::endl;
   //visualize_pcd(cloud_filtered, 0, 255, 0);
/****************************************************************************************/
   // RANSAC平面拟合(segmention)
   pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
   pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
   pcl::SACSegmentation<PointXYZIR> seg;      //创建分割对象
   seg.setOptimizeCoefficients(true);
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType(pcl::SAC_RANSAC);
   seg.setMaxIterations(1000);
   seg.setDistanceThreshold(plane_line_dist_threshold.first); //设置距离阈值
   seg.setInputCloud(cloud_filtered);
   //seg.setInputCloud(cloud_passthrough);
   seg.segment(*inliers, *coefficients);
   //coefficients即为拟合平面的法向量 // mag标定板法向量的大小
   //保存平面拟合后点点云数据
   pcl::PointCloud<PointXYZIR>::Ptr cloud_seg(new pcl::PointCloud<PointXYZIR>);
   pcl::ExtractIndices<PointXYZIR> extract;
   extract.setInputCloud(cloud_filtered);
   //extract.setInputCloud(cloud_passthrough);
   extract.setIndices(inliers);
   extract.setNegative(false);
   extract.filter(*cloud_seg);
   std::cout << "RANSAC平面拟合后点云数：" << cloud_seg->points.size() << std::endl;
   std::cout << "Model coefficients: " << coefficients->values[0] << " "
		                                 << coefficients->values[1] << " "
		                                 << coefficients->values[2] << " "
		                                 << coefficients->values[3] << std::endl;          //平面方程 ax+by+cz+d = 0
   
   // pcl::StatisticalOutlierRemoval<PointXYZIR> sor;   //创建滤波器对象
   // pcl::PointCloud<PointXYZIR>::Ptr statistical_filtered(new pcl::PointCloud<PointXYZIR>);
   // sor.setInputCloud (cloud_seg);              //设置待滤波的点云
   // sor.setMeanK (50);                               //设置在进行统计时考虑的临近点个数
   // sor.setStddevMulThresh (1.0);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
   // sor.filter (*statistical_filtered);                    //滤波结果存储到cloud_filtered
   // std::cout << "剔除离群后点云数：" << statistical_filtered->points.size() << std::endl;
   // visualize_pcd(statistical_filtered, 0, 255, 0);

   //将点云投射到拟合的平面上
   pcl::PointCloud<PointXYZIR>::Ptr cloud_projected(new pcl::PointCloud<PointXYZIR>);
   pcl::ProjectInliers<PointXYZIR> proj;
   proj.setModelType(pcl::SACMODEL_PLANE);
   proj.setInputCloud(cloud_seg);
   proj.setModelCoefficients(coefficients);
   proj.filter(*cloud_projected);
   std::cout << "点云投射到拟合平面后点云数：" << cloud_projected->points.size() << std::endl;
   //可视化拟合平面点云
   //visualize_pcd(cloud_projected, 0, 255, 0);

   // //将标定板拟合点云存入vector中,待后续投影
   std::vector<cv::Point3f> tmp;
   for(size_t i = 0; i < (int)cloud_projected->points.size(); i++){
      tmp.push_back(cv::Point3f(cloud_projected->points[i].x * 1000, cloud_projected->points[i].y * 1000, cloud_projected->points[i].z * 1000)); //单位：mm
   }
   chessBoard_lidar_points.push_back(tmp);
/****************************************************************投影到平面*************************************************************************************/
   Eigen::Vector3f n((float)coefficients->values[0], (float)coefficients->values[1], (float)coefficients->values[2]);
   Eigen::Vector3f i(0.0f, 0.0f, 1.0f);
   n.normalize();
   Eigen::Matrix3f r = Eigen::Quaternionf::FromTwoVectors(n, i).toRotationMatrix();
   pcl::PointCloud<PointXYZIR>::Ptr proj_point(new pcl::PointCloud<PointXYZIR>);
   for(int i =0; i<(int)cloud_projected->points.size(); i++)
   {
      //std::cout << "x before: " << cloud_projected->points[i].x << " y before: " << cloud_projected->points[i].y << " z before: " << cloud_projected->points[i].z <<std::endl;
      Eigen::Vector3f p((float)cloud_projected->points[i].x, (float)cloud_projected->points[i].y, (float)cloud_projected->points[i].z);
      Eigen::Vector3f proj = r * p;
      //std::cout << "proj: " << proj.segment<3>(0).transpose() << std::endl;
      PointXYZIR point = {proj[0], proj[1], proj[2], cloud_projected->points[i].intensity, cloud_projected->points[i].ring};
      proj_point->push_back(point);
   }
   //visualize_pcd(proj_point, 0, 0, 255);

/**************************************************************************************************************************************************************/
   std::vector<pcl::PointCloud<PointXYZIR>::Ptr> all_proj_line;
   std::vector<pcl::PointCloud<PointXYZIR>::Ptr> all_proj_line_t;
   std::vector<float> all_M_distance;
   std::vector<float> all_sita;
   std::vector<float> all_min_x;
   std::vector<float> all_min_y;
   std::vector<float> all_max_x;
   std::vector<float> all_max_y;
   //找Rotate
   for(int seta = 1; seta < 90; seta++)
   {
      pcl::PointCloud<PointXYZIR>::Ptr proj_line(new pcl::PointCloud<PointXYZIR>);
      float sita = seta*pi/180;      //内置函数的返回值是弧度，要先把角度换成弧度 弧度 = 角度*pi/180
      float k = tanf((float)sita);
      std::vector<float> ds;

      //std::cout << "seta: " << seta << std::endl;
      //std::cout << "sita: " << sita << std::endl;
      for(int i =0; i<(int)proj_point->points.size(); i++)
      {
         //std::cout << "x before: " << proj_point->points[i].x << " y before: " << proj_point->points[i].y  <<std::endl;
         float x = (float)((k*1 + (proj_point->points[i].x)/k + proj_point->points[i].y -k)/(1/k + k));
         float y = (float)(-1/k*(x - proj_point->points[i].x) + proj_point->points[i].y);
         //std::cout << "proj_line_x: " << x << " "<<  "proj_line_y: "  << y <<  " "<<  "proj_line_z: "  << proj_point->points[i].z << std::endl;
         PointXYZIR point = {x, y, proj_point->points[i].z , proj_point->points[i].intensity, proj_point->points[i].ring};
         proj_line->push_back(point);
      }
      //visualize_pcd2(proj_point, proj_line);
      PointXYZIR min, max;
      pcl::getMinMax3D(*proj_line, min, max);
      float distance = (max.x - min.x)/cos(sita);
      float M_distance = std::abs(distance - 1);
      float min_x = min.x;
      float max_x = max.x;
      // std::cout << "0-90min.x = " << min.x << std::endl;
      // std::cout << "0-90max.x = " << max.x << std::endl;
      // std::cout << "0-90distance = " << distance << std::endl;
      // std::cout << "0-90M_distance = " << M_distance << std::endl;

      if (0.8 < distance && distance < 1.2)
      {
         pcl::PointCloud<PointXYZIR>::Ptr proj_line_t(new pcl::PointCloud<PointXYZIR>);
         for(int i =0; i<(int)proj_point->points.size(); i++)
         {
            //std::cout << "x before: " << proj_point->points[i].x << " y before: " << proj_point->points[i].y  <<std::endl;
            float x = (float)(((-1/k)*1 + (proj_point->points[i].x)/(-1/k) + proj_point->points[i].y -(-1/k))/(1/(-1/k) + (-1/k)));
            float y = (float)(-1/(-1/k)*(x - proj_point->points[i].x) + proj_point->points[i].y);
            //std::cout << "proj_line_x: " << x << " "<<  "proj_line_y: "  << y <<  " "<<  "proj_line_z: "  << proj_point->points[i].z << std::endl;
            PointXYZIR point = {x, y, proj_point->points[i].z , proj_point->points[i].intensity, proj_point->points[i].ring};
            proj_line_t->push_back(point);
         }
         PointXYZIR min, max;
         pcl::getMinMax3D(*proj_line_t, min, max);    //min为所有点中最小的x,y,z  max为所有点中最小的x,y,z
         float distance_t = (max.x - min.x)/cos((90-seta)*pi/180);
         float M_distance_t = std::abs(distance_t - 1);
         float min_y = min.x;
         float max_y = max.x;

         // std::cout << "min.x = " << min.x << std::endl;
         // std::cout << "max.x = " << max.x << std::endl;
         // std::cout << "distance = " << distance_t << std::endl;
         // std::cout << "M_distance_t = " << M_distance_t << std::endl;

         //visualize_pcd2(proj_point, proj_line);

         if (0.8 < distance_t && distance_t < 1.2 )
         {
            float A_distance_t = M_distance + M_distance_t;
            all_proj_line.push_back(proj_line);
            all_proj_line_t.push_back(proj_line_t);
            all_M_distance.push_back(A_distance_t);
            all_sita.push_back(sita);
            all_min_x.push_back(min_x);    //0-90度验证   x最小
            all_min_y.push_back(min_y);    //90-180度验证 x最小
            all_max_x.push_back(max_x);    //0-90度验证   x最大
            all_max_y.push_back(max_y);    //90-180度验证 x最大
         }
    
      }

   }

   if(!all_M_distance.empty())
   {
      std::vector<float>::iterator smallest = std::min_element(all_M_distance.begin(), all_M_distance.end());
      int indice = std::distance(all_M_distance.begin(), smallest);
      //std::cout << "size is " << all_M_distance.size() << " at position " << indice << std::endl;
      //std::cout << "Min element is " << all_M_distance[indice] << " at position " << indice << std::endl;
      //std::cout << "min_x: " <<  all_min_x[indice] << " min_y: " << all_min_y[indice] << " max_x:" <<  all_max_x[indice] << " max_y: " << all_max_y[indice] << " sita: " <<  all_sita[indice] << std::endl;
      //std::cout << proj_point->points.size() << std::endl;
      //visualize_pcd3(all_proj_line[indice], all_proj_line_t[indice], proj_point);

      //找中心点translation
      float k_x = tanf((float)(all_sita[indice]));
      float middle_x_x = (all_max_x[indice] - all_min_x[indice])/2 + all_min_x[indice];
      float middle_x_y = k_x * middle_x_x ;
      //std::cout << "middle_x_x:" << middle_x_x << " " << "middle_x_y:" << middle_x_y  << " " << "k_x:" << k_x << std::endl;
      float k_y = -1/k_x;
      float middle_y_x = (all_max_y[indice] - all_min_y[indice])/2 + all_min_y[indice];
      float middle_y_y = k_y * middle_y_x ;
      //std::cout << "middle_y_x:" << middle_y_x  << " "<< "middle_y_y:" << middle_y_y << " "  << "k_y:" << k_y << std::endl;
      float b_x = middle_x_y - ((k_y) * middle_x_x);
      float b_y = middle_y_y - ((k_x) * middle_y_x);
      float centroid_x = (b_x - b_y)/(k_x + (-k_y));
      float centroid_y = ((k_y) * centroid_x) + b_x;
      //std::cout << "centroid_x: " << centroid_x << " " << "centroid_y: " << centroid_y;
   
      Eigen::Matrix3f rotate;
      std::cout << all_sita[indice] << std::endl;
      rotate << cos(all_sita[indice]) , -sin(all_sita[indice]), 0 ,
                sin(all_sita[indice]) ,  cos(all_sita[indice]), 0 ,
                0 ,                      0 ,                    1;
      std::cout << "rotate: " <<std::endl << rotate << std::endl;
      Eigen::Matrix3f translation;
      translation << 1, 0, centroid_x , 
                     0, 1, centroid_y , 
                     0, 0,          1 ;
      std::cout << "translation: " << std::endl << translation << std::endl;

      //原点为中心定义1X1正方形
      Eigen::Vector3f left_up   (-0.5,  0.5, 1);
      Eigen::Vector3f left_down (-0.5, -0.5, 1);
      Eigen::Vector3f right_up  ( 0.5,  0.5, 1);
      Eigen::Vector3f right_down( 0.5, -0.5, 1);
      Eigen::Vector3f center    (   0,    0, 1);
      // //旋转
      left_up    =  (rotate * left_up);
      left_down  =  (rotate * left_down);
      right_up   =  (rotate * right_up);
      right_down =  (rotate * right_down);
      center     =  (rotate * center);
      //平移
      left_up    =  (translation * left_up);
      left_down  =  (translation * left_down);
      right_up   =  (translation * right_up);
      right_down =  (translation * right_down);
      center     =  (translation * center);

      //平面角点
      pcl::PointCloud<PointXYZIR>::Ptr corner_point(new pcl::PointCloud<PointXYZIR>);
      PointXYZIR left_up_point    = {left_up[0],    left_up[1],    proj_point->points[0].z , proj_point->points[0].intensity, proj_point->points[0].ring};
      PointXYZIR left_down_point  = {left_down[0],  left_down[1],  proj_point->points[0].z , proj_point->points[0].intensity, proj_point->points[0].ring};
      PointXYZIR right_up_point   = {right_up[0],   right_up[1],   proj_point->points[0].z , proj_point->points[0].intensity, proj_point->points[0].ring};
      PointXYZIR right_down_point = {right_down[0], right_down[1], proj_point->points[0].z , proj_point->points[0].intensity, proj_point->points[0].ring};
      PointXYZIR center_point     = {center[0],     center[1],     proj_point->points[0].z , proj_point->points[0].intensity, proj_point->points[0].ring};
      corner_point->push_back(left_up_point);
      corner_point->push_back(left_down_point);
      corner_point->push_back(right_up_point);
      corner_point->push_back(right_down_point);
      corner_point->push_back(center_point);

      //visualize_box(proj_point, left_up_point, left_down_point, right_up_point, right_down_point);

      //转回三维角点坐标
      Eigen::Vector3f twoD_left_up     (left_up[0],    left_up[1],    proj_point->points[0].z);
      Eigen::Vector3f twoD_left_down   (left_down[0],  left_down[1],  proj_point->points[0].z);
      Eigen::Vector3f twoD_right_up    (right_up[0],   right_up[1],   proj_point->points[0].z);
      Eigen::Vector3f twoD_right_down  (right_down[0], right_down[1], proj_point->points[0].z);
      Eigen::Vector3f twoD_center      (center[0],     center[1],     proj_point->points[0].z);

      twoD_left_up = r.inverse() * twoD_left_up;
      twoD_left_down = r.inverse() * twoD_left_down;
      twoD_right_up = r.inverse() * twoD_right_up;
      twoD_right_down = r.inverse() * twoD_right_down;
      twoD_center = r.inverse() * twoD_center;

      pcl::PointCloud<PointXYZIR>::Ptr all_corner(new pcl::PointCloud<PointXYZIR>);
      PointXYZIR threed_left_up_point    = {twoD_left_up[0],    twoD_left_up[1],    twoD_left_up[2] ,   0, 0};
      PointXYZIR threed_left_down_point  = {twoD_left_down[0],  twoD_left_down[1],  twoD_left_down[2] , 0, 0};
      PointXYZIR threed_right_up_point   = {twoD_right_up[0],   twoD_right_up[1],   twoD_right_up[2] ,  0, 0};
      PointXYZIR threed_right_down_point = {twoD_right_down[0], twoD_right_down[1], twoD_right_down[2] ,0, 0};
      PointXYZIR threed_center_point     = {twoD_center[0],     twoD_center[1],     twoD_center[2] ,    0, 0};

      all_corner->push_back(threed_left_up_point);    
      all_corner->push_back(threed_left_down_point);  
      all_corner->push_back(threed_right_up_point);   
      all_corner->push_back(threed_right_down_point); 
      all_corner->push_back(threed_center_point);
      //找上下左右点
      std::vector<float> t1;
      std::vector<float> t2;
      std::vector<float> t3;
      std::vector<float> t4;

      t1.push_back((float)(threed_left_up_point.x/threed_left_up_point.y));
      t1.push_back((float)(threed_left_up_point.y/threed_left_up_point.y));
      t1.push_back((float)(threed_left_up_point.z/threed_left_up_point.y));

      t2.push_back((float)(threed_left_down_point.x/threed_left_down_point.y));
      t2.push_back((float)(threed_left_down_point.y/threed_left_down_point.y));
      t2.push_back((float)(threed_left_down_point.z/threed_left_down_point.y));

      t3.push_back((float)(threed_right_up_point.x/threed_right_up_point.y));
      t3.push_back((float)(threed_right_up_point.y/threed_right_up_point.y));
      t3.push_back((float)(threed_right_up_point.z/threed_right_up_point.y));

      t4.push_back((float)(threed_right_down_point.x/threed_right_down_point.y));
      t4.push_back((float)(threed_right_down_point.y/threed_right_down_point.y));
      t4.push_back((float)(threed_right_down_point.z/threed_right_down_point.y));

      std::vector<float> all_z;
      std::vector<float> all_x;
      all_z.push_back(t1[2]);
      all_z.push_back(t2[2]);
      all_z.push_back(t3[2]);
      all_z.push_back(t4[2]);

      all_x.push_back(t1[0]);
      all_x.push_back(t2[0]);
      all_x.push_back(t3[0]);
      all_x.push_back(t4[0]);

      std::vector<float>::iterator z_min = std::min_element(all_z.begin(), all_z.end());
      std::vector<float>::iterator z_max = std::max_element(all_z.begin(), all_z.end());
      std::vector<float>::iterator x_min = std::min_element(all_x.begin(), all_x.end());
      std::vector<float>::iterator x_max = std::max_element(all_x.begin(), all_x.end());
      int down_indic = std::distance(all_z.begin(), z_min);
      int up_indic = std::distance(all_z.begin(), z_max);
      int left_indic = std::distance(all_x.begin(), x_min);
      int right_indic = std::distance(all_x.begin(), x_max);
      std::cout << down_indic << up_indic << left_indic << right_indic << std::endl;

      pcl::PointCloud<PointXYZIR>::Ptr left_right_down_up_con(new pcl::PointCloud<PointXYZIR>);
      left_right_down_up_con->push_back(all_corner->points[left_indic]);
      left_right_down_up_con->push_back(all_corner->points[right_indic]);
      left_right_down_up_con->push_back(all_corner->points[down_indic]);
      left_right_down_up_con->push_back(all_corner->points[up_indic]);
      left_right_down_up_con->push_back(all_corner->points[4]);

      //visualize_pcd2(left_right_down_up_con, cloud_projected);
      lidar_board_coenerAndCenter.push_back(cv::Point3f(all_corner->points[left_indic].x * 1000, all_corner->points[left_indic].y * 1000, all_corner->points[left_indic].z * 1000)); //右
      lidar_board_coenerAndCenter.push_back(cv::Point3f(all_corner->points[right_indic].x * 1000, all_corner->points[right_indic].y * 1000, all_corner->points[right_indic].z * 1000)); //左
      lidar_board_coenerAndCenter.push_back(cv::Point3f(all_corner->points[up_indic].x * 1000, all_corner->points[up_indic].y * 1000, all_corner->points[up_indic].z * 1000)); //下
      lidar_board_coenerAndCenter.push_back(cv::Point3f(all_corner->points[down_indic].x * 1000, all_corner->points[down_indic].y * 1000, all_corner->points[down_indic].z * 1000)); //上
      lidar_board_coenerAndCenter.push_back(cv::Point3f(all_corner->points[4].x * 1000, all_corner->points[4].y * 1000, all_corner->points[4].z * 1000)); //中

      all_proj_line.clear();
      all_proj_line_t.clear();
      all_M_distance.clear();
      all_sita.clear();
      all_min_x.clear();
      all_min_y.clear();
      all_max_x.clear();
      all_max_y.clear();
      all_proj_line.clear();
      all_proj_line_t.clear();

      return 1;

   }

   else return 0;

};
/**********************************************************************************************************************************/

//    // 寻找标定板上每条扫描线的最大点和最小点，即与标定板边线的交点
//    // 将每个点云按照Scan number进行归类
//    std::vector<std::deque<PointXYZIR*> > candidate_segments(lidar_ring_count);
//    for (size_t i = 0; i < cloud_projected->points.size(); i++) {
//       //int ring_number = (int)cloud_projected->points[i].ring; //这里是个坑，需要注意，速腾lidar32 Scan的ID为：1->32，所有需要减去1
//       int ring_number = (int)cloud_projected->points[i].ring -1; //这里是个坑，需要注意，速腾lidar32 Scan的ID为：1->32，所有需要减去1
//       candidate_segments[ring_number].push_back(&(cloud_projected->points[i]));
//    }

//    // Second: Arrange points in every ring in descending order of y coordinate
//    pcl::PointCloud<PointXYZIR>::Ptr min_points(new pcl::PointCloud<PointXYZIR>);        // 标定板右边界激光点集合
//    pcl::PointCloud<PointXYZIR>::Ptr max_points(new pcl::PointCloud<PointXYZIR>);        // 标定板左边界激光点集合
//    // int maxLidarPoints_scanNumber = 0; //用于记录点云最多的那条scan的点云数量
//    // double y_right = 9999.0, y_left = -9999.0, z_right, z_left;   // z_right为最右侧边界点的z值， z_left为最左侧边界点的z值。

//    for (int i = 0; i < (int)candidate_segments.size(); i++) 
//    {
//       if (candidate_segments[i].size() == 0 || candidate_segments[i].size() == 1) // 等于0说明这条扫描线上没有点，1说明这条线上只有1个点,
//       {
//          continue;
//       }
//       // // 寻找标定板上每条Scan上的左右两个边界点
//       // double y_min = 9999.0;
//       // double y_max = -9999.0;
//       // int y_min_index, y_max_index;
//       // for (int p = 0; p < candidate_segments[i].size(); p++) {
//       //    if (candidate_segments[i][p]->y > y_max) {
//       //       y_max = candidate_segments[i][p]->y;
//       //       y_max_index = p;
//       //    }
//       //    if (candidate_segments[i][p]->y < y_min) {
//       //       y_min = candidate_segments[i][p]->y;
//       //       y_min_index = p;
//       //    }
//       // }

//       // min_points->push_back(*candidate_segments[i][y_min_index]); // 标定板右边界激光点集合
//       // max_points->push_back(*candidate_segments[i][y_max_index]); // 标定板左边界激光点集合
//       // 寻找标定板上每条Scan上的左右两个边界点
//       double x_min = 9999.0;
//       double x_max = -9999.0;
//       int x_min_index, x_max_index;
//       for (int p = 0; p < candidate_segments[i].size(); p++) {
//          if (candidate_segments[i][p]->x > x_max) {
//             x_max = candidate_segments[i][p]->x;
//             x_max_index = p;
//          }
//          if (candidate_segments[i][p]->x < x_min) {
//             x_min = candidate_segments[i][p]->x;
//             x_min_index = p;
//          }
//       }

//       min_points->push_back(*candidate_segments[i][x_min_index]); // 标定板右边界激光点集合
//       max_points->push_back(*candidate_segments[i][x_max_index]); // 标定板左边界激光点集合

//       //寻找点云最多的那条scan及其对应左右边界点
//       // if ((int)candidate_segments[i].size() > maxLidarPoints_scanNumber)
//       // {
//       //    maxLidarPoints_scanNumber = (int)candidate_segments[i].size();
//       //    z_right = candidate_segments[i][y_min_index]->z;
//       //    z_left = candidate_segments[i][y_max_index]->z;
//       // }
//    }
//    // std::cout << "标定板右边界激光点集合" << std::endl;
//    // visualize_pcd(min_points); // 标定板右边界激光点集合
//    // std::cout << "标定板左边界激光点集合" << std::endl;
//    // visualize_pcd(max_points); // 标定板左边界激光点集合

//    // double y_right = 9999.0, y_left = -9999.0, z_right, z_left;   // z_right为最右侧边界点的z值， z_left为最左侧边界点的z值。
//    // for(size_t i = 0; i < (int)min_points->points.size(); i++)
//    // {
//    //    if (min_points->points[i].y < y_right)
//    //    {
//    //       y_right = min_points->points[i].y;
//    //       z_right = min_points->points[i].z;
//    //    }
//    // }
//    // for(size_t i = 0; i < (int)max_points->points.size(); i++)
//    // {
//    //    if (max_points->points[i].y > y_left)
//    //    {
//    //       y_left = max_points->points[i].y;
//    //       z_left = max_points->points[i].z;
//    //    }
//    // }

//    double x_right = 9999.0, x_left = -9999.0, z_right, z_left;   // z_right为最右侧边界点的z值， z_left为最左侧边界点的z值。
//    for(size_t i = 0; i < (int)min_points->points.size(); i++)
//    {
//       if (min_points->points[i].x < x_right)
//       {
//          x_right = min_points->points[i].x;
//          z_right = min_points->points[i].z;
//       }
//    }
//    for(size_t i = 0; i < (int)max_points->points.size(); i++)
//    {
//       if (max_points->points[i].x > x_left)
//       {
//          x_left = max_points->points[i].x;
//          z_left = max_points->points[i].z;
//       }
//    }

//    pcl::PointCloud<PointXYZIR>::Ptr left_up_points(new pcl::PointCloud<PointXYZIR>);    // 标定板左上边界激光点集合
//    pcl::PointCloud<PointXYZIR>::Ptr left_down_points(new pcl::PointCloud<PointXYZIR>);  // 标定板左下边界激光点集合
//    pcl::PointCloud<PointXYZIR>::Ptr right_up_points(new pcl::PointCloud<PointXYZIR>);   // 标定板右上边界激光点集合
//    pcl::PointCloud<PointXYZIR>::Ptr right_down_points(new pcl::PointCloud<PointXYZIR>); // 标定板右下边界激光点集合

//    for(int i = 0; i < (int)min_points->points.size(); i++){
//       // 右上边界点
//       if(min_points->points[i].z >= z_right){
//          right_up_points->push_back(min_points->points[i]);
//       }
//       // 右下边界点
//       if(min_points->points[i].z <= z_right){
//          right_down_points->push_back(min_points->points[i]);
//       }
//    }
//    for (int i = 0; i < (int)max_points->points.size(); i++)
//    {
//       // 左上边界点
//       if(max_points->points[i].z >= z_left){
//          left_up_points->push_back(max_points->points[i]);
//       }
//       // 左下边界点
//       if(max_points->points[i].z <= z_left){
//          left_down_points->push_back(max_points->points[i]);
//       }
//    }
//    std::cout << "标定板右上界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, right_up_points);
//    std::cout << "标定板右下界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, right_down_points);
//    std::cout << "标定板左上界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, left_up_points);
//    std::cout << "标定板左下界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, left_down_points);
// /**************************************离群点滤波**************************************/
//    pcl::StatisticalOutlierRemoval<PointXYZIR> sor1;   //创建滤波器对象
//    pcl::PointCloud<PointXYZIR>::Ptr cloud_filtered1(new pcl::PointCloud<PointXYZIR>);
//    sor1.setInputCloud (right_up_points);              //设置待滤波的点云
//    sor1.setMeanK (50);                               //设置在进行统计时考虑的临近点个数
//    sor1.setStddevMulThresh (0.5);                      //设置判断是否为离群点的阀值，用来倍乘标准差，也就是上面的std_mul
//    sor1.filter (*cloud_filtered1);                    //滤波结果存储到cloud_filtered

//    pcl::StatisticalOutlierRemoval<PointXYZIR> sor2;   
//    pcl::PointCloud<PointXYZIR>::Ptr cloud_filtered2(new pcl::PointCloud<PointXYZIR>);
//    sor2.setInputCloud (right_down_points);                         
//    sor2.setMeanK (50);                             
//    sor2.setStddevMulThresh (0.5);                     
//    sor2.filter (*cloud_filtered2);                  

//    pcl::StatisticalOutlierRemoval<PointXYZIR> sor3;  
//    pcl::PointCloud<PointXYZIR>::Ptr cloud_filtered3(new pcl::PointCloud<PointXYZIR>);
//    sor3.setInputCloud (left_up_points);                          
//    sor3.setMeanK (50);                              
//    sor3.setStddevMulThresh (0.5);                     
//    sor3.filter (*cloud_filtered3);                    

//    pcl::StatisticalOutlierRemoval<PointXYZIR> sor4;   
//    pcl::PointCloud<PointXYZIR>::Ptr cloud_filtered4(new pcl::PointCloud<PointXYZIR>);
//    sor4.setInputCloud (left_down_points);                           
//    sor4.setMeanK (50);                              
//    sor4.setStddevMulThresh (0.5);                    
//    sor4.filter (*cloud_filtered4);                    

//    std::cout << "去除噪点后标定板右上界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, cloud_filtered1);
//    std::cout << "去除噪点后标定板右下界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, cloud_filtered2);
//    std::cout << "去除噪点后标定板左上界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, cloud_filtered3);
//    std::cout << "去除噪点后标定板左下界激光点集合" << std::endl;
//    visualize_pcd2(cloud_projected, cloud_filtered4);
// /*****************************************************************************/
//    // Fit lines through minimum and maximum points //直线拟合
//    pcl::ModelCoefficients::Ptr coefficients_left_up(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_left_up(new pcl::PointIndices);

//    pcl::ModelCoefficients::Ptr coefficients_left_dwn(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_left_dwn(new pcl::PointIndices);

//    pcl::ModelCoefficients::Ptr coefficients_right_up(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_right_up(new pcl::PointIndices);

//    pcl::ModelCoefficients::Ptr coefficients_right_dwn(new pcl::ModelCoefficients);
//    pcl::PointIndices::Ptr inliers_right_dwn(new pcl::PointIndices);

//    seg.setModelType(pcl::SACMODEL_LINE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setDistanceThreshold(plane_line_dist_threshold.second);

//    //seg.setInputCloud(left_up_points);
//    seg.setInputCloud(cloud_filtered3);
//    seg.segment(*inliers_left_up, *coefficients_left_up); // Fitting line1 through max points
//    if (inliers_left_up->indices.size () == 0)
//    {
//       PCL_ERROR ("Could not estimate inliers_left_up model for the given dataset.");
//       return (-1);
//    }
//    // std::cout << coefficients_left_up->values.size() << std::endl;
//    // std::cout << coefficients_left_up->values[0] << "," << coefficients_left_up->values[1] << ","
//    //           << coefficients_left_up->values[2] << "," << coefficients_left_up->values[3] << ","
//    //           << coefficients_left_up->values[4] << std::endl;

//    //seg.setInputCloud(left_down_points);
//    seg.setInputCloud(cloud_filtered4);
//    seg.segment(*inliers_left_dwn, *coefficients_left_dwn); // Fitting line2 through max points
//    if (inliers_left_dwn->indices.size () == 0)
//    {
//       PCL_ERROR ("Could not estimate inliers_left_dwn model for the given dataset.");
//       return (-1);
//    }
//    // std::cout << coefficients_left_dwn->values.size() << std::endl;
//    // std::cout << coefficients_left_dwn->values[0] << "," << coefficients_left_dwn->values[1] << ","
//    //           << coefficients_left_dwn->values[2] << "," << coefficients_left_dwn->values[3] << ","
//    //           << coefficients_left_dwn->values[4] << std::endl;

//    //seg.setInputCloud(right_up_points);
//    seg.setInputCloud(cloud_filtered1);
//    seg.segment(*inliers_right_up, *coefficients_right_up); // Fitting line1 through min points
//    if (inliers_right_up->indices.size () == 0)
//    {
//       PCL_ERROR ("Could not estimate inliers_right_up model for the given dataset.");
//       return (-1);
//    }
//    // std::cout << coefficients_right_up->values.size() << std::endl;
//    // std::cout << coefficients_right_up->values[0] << "," << coefficients_right_up->values[1] << ","
//    //           << coefficients_right_up->values[2] << "," << coefficients_right_up->values[3] << ","
//    //           << coefficients_right_up->values[4] << std::endl;

//    //seg.setInputCloud(right_down_points);
//    seg.setInputCloud(cloud_filtered2);
//    seg.segment(*inliers_right_dwn, *coefficients_right_dwn); // Fitting line2 through min points
//    if (inliers_right_dwn->indices.size () == 0)
//    {
//       PCL_ERROR ("Could not estimate inliers_right_dwn model for the given dataset.");
//       return (-1);
//    }
//    // std::cout << coefficients_right_dwn->values.size() << std::endl;
//    // std::cout << coefficients_right_dwn->values[0] << "," << coefficients_right_dwn->values[1] << ","
//    //           << coefficients_right_dwn->values[2] << "," << coefficients_right_dwn->values[3] << ","
//    //           << coefficients_right_dwn->values[4] << std::endl;

//    // Find out 2 (out of the four) intersection points
//    Eigen::Vector4f Point_l;
//    pcl::PointCloud<PointXYZIR>::Ptr basic_cloud_ptr(new pcl::PointCloud<PointXYZIR>);
//    PointXYZIR basic_point; // intersection points stored here
//    // std::cout << 1 << std::endl;
//    if (pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_left_dwn, Point_l)) { // 左边点
//       //std::cout << "左边点" << std::endl;
//       basic_point.x = Point_l[0];
//       basic_point.y = Point_l[1];
//       basic_point.z = Point_l[2];
//       basic_cloud_ptr->points.push_back(basic_point);
//       lidar_board_coenerAndCenter.push_back(cv::Point3f(Point_l[0] * 1000, Point_l[1] * 1000, Point_l[2] * 1000)); //与图像坐标系单位统一，mm
//    }
//    // std::cout << 2 << std::endl;
//    if (pcl::lineWithLineIntersection(*coefficients_right_up, *coefficients_right_dwn, Point_l)) { // 右边点
//       //std::cout << "右边点" << std::endl;
//       basic_point.x = Point_l[0];
//       basic_point.y = Point_l[1];
//       basic_point.z = Point_l[2];
//       basic_cloud_ptr->points.push_back(basic_point);
//       lidar_board_coenerAndCenter.push_back(cv::Point3f(Point_l[0] * 1000, Point_l[1] * 1000, Point_l[2] * 1000));
//    }
//    // std::cout << 3 << std::endl;
//    if (pcl::lineWithLineIntersection(*coefficients_left_dwn, *coefficients_right_dwn, Point_l)) { // 下面点
//       //std::cout << "下面点" << std::endl;
//       basic_point.x = Point_l[0];
//       basic_point.y = Point_l[1];
//       basic_point.z = Point_l[2];
//       basic_cloud_ptr->points.push_back(basic_point);
//       lidar_board_coenerAndCenter.push_back(cv::Point3f(Point_l[0] * 1000, Point_l[1] * 1000, Point_l[2] * 1000));
//    }
//    // std::cout << 4 << std::endl;
//    if (pcl::lineWithLineIntersection(*coefficients_left_up, *coefficients_right_up, Point_l)) { // 上面点
//       //std::cout << "上面点" << std::endl;
//       basic_point.x = Point_l[0];
//       basic_point.y = Point_l[1];
//       basic_point.z = Point_l[2];
//       basic_cloud_ptr->points.push_back(basic_point);
//       lidar_board_coenerAndCenter.push_back(cv::Point3f(Point_l[0] * 1000, Point_l[1] * 1000, Point_l[2] * 1000));
//    }
//    // std::cout << 5 << std::endl;

//    // 标定板激光点云拟合平面中心点坐标求解
//    PointXYZIR velodynepoint;
//    velodynepoint.x = (basic_cloud_ptr->points[0].x + basic_cloud_ptr->points[1].x) / 2;
//    velodynepoint.y = (basic_cloud_ptr->points[0].y + basic_cloud_ptr->points[1].y) / 2;
//    velodynepoint.z = (basic_cloud_ptr->points[0].z + basic_cloud_ptr->points[1].z) / 2;
//    basic_cloud_ptr->points.push_back(velodynepoint);
//    lidar_board_coenerAndCenter.push_back(cv::Point3f(velodynepoint.x * 1000, velodynepoint.y * 1000, velodynepoint.z * 1000)); //与图像坐标系单位统一，mm

//    // // 标定板激光点云拟合平面 + 四个边角点 + 中心点 可视化
//    visualize_pcd2(cloud_projected ,basic_cloud_ptr);

//    // 检测求出的四个标定板边角点是否符合要求
//    double line_left_down_dist = abs(pcl::euclideanDistance(basic_cloud_ptr->points[0], basic_cloud_ptr->points[2])-1);
//    double line_left_up_dist = abs(pcl::euclideanDistance(basic_cloud_ptr->points[0], basic_cloud_ptr->points[3]) -1);
//    double line_right_down_dist = abs(pcl::euclideanDistance(basic_cloud_ptr->points[1], basic_cloud_ptr->points[2]) -1);
//    double line_right_up_dist = abs(pcl::euclideanDistance(basic_cloud_ptr->points[1], basic_cloud_ptr->points[3]) -1);
//    if(cv::max(line_left_down_dist, line_left_up_dist) <= line_fit2real_dist && cv::max(line_right_down_dist, line_right_up_dist) <= line_fit2real_dist){
//       std::cout << "lidar:" << lidar_board_coenerAndCenter.size() << std::endl;
//       return 1;
//    }
//    else return 0;

// };



int CALIB::Project3DPoints(std::vector<std::vector<cv::Point3f> > chessBoard_lidar_points, cv::Mat R_camToLidar, cv::Mat T_camToLidar, std::string srcImg_path, cv::Mat cameramat, cv::Mat distcoeff, std::vector<std::string> img_files, std::string img_projection_path)
{
   for(size_t i = 0; i < (int)chessBoard_lidar_points.size(); i++)
   {
      
      std::vector<cv::Point2f> lidarFeaturesToImg;
      if(chessBoard_lidar_points[i].empty())
      {
         continue;
      }
      cv::projectPoints(chessBoard_lidar_points[i], R_camToLidar, T_camToLidar, cameramat, distcoeff, lidarFeaturesToImg);
      // std::cout << "!!! : " << i_params.cameramat << std::endl;
      // std::cout << "!!! : " << i_params.distcoeff << std::endl;
      // std::cout << lidarFeaturesToImg << endl;

      std::cout << "project onto : "<< img_files[i] << std::endl;
      cv::Mat show_img_src = cv::imread(srcImg_path + img_files[i], 1); //读取原始图像
      cv::Mat show_img_dst;
      cv::undistort(show_img_src, show_img_dst, cameramat, distcoeff, cameramat); //图像畸变矫正
      for (int j = 0; j < lidarFeaturesToImg.size(); j++)
      {
         cv::Point2f p = lidarFeaturesToImg[j];
         cv::circle(show_img_src, p, 2, CV_RGB(255, 0, 0), -1, 8, 0); //激光点在图像上为红色
      };
      cv::imshow("show_img_dst", show_img_src);
      cv::waitKey(1000);
      cv::destroyAllWindows();

      cv::imwrite(img_projection_path + img_files[i], show_img_src); //保存投影图像

   };

   return 0;
};

