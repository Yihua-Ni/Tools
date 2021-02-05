#ifndef VIEWER_H
#define VIEWER_H

#include <Eigen/Core>
#include <deque>
#include <unordered_map>
#include <pangolin/pangolin.h>
#include <thread>
#include <opencv2/opencv.hpp>

class Viewer
{
public:
    Viewer();
    ~Viewer();
    void init();
    void stop();
    bool isStop();
    void insertPoints(const double& time, const std::vector<Eigen::Vector3f>& p3ds, std::string str = "map_points");
    void insertImg(const cv::Mat& im);
    bool bShowIpm_;

private:
    void run();
    void drawImage(cv::Mat &im, bool &bUpdate);

	std::string win_name_;
	int win_w_;
	int win_h_;
	bool bStop_;
	bool bFollow_;


    double dur_;
    double curr_t_;

    std::unordered_map<std::string, std::deque<std::pair<double, std::vector<Eigen::Vector3f>>>>  pts_;
    std::unordered_map<std::string, Eigen::Vector3f> colors_;
    std::deque<cv::Mat> imgs_;

    std::mutex pts_mtx_;
    std::mutex img_mtx_;

	std::thread th_draw_;
};

#endif