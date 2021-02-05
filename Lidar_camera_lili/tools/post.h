#ifndef POST_H
#define POST_H
#include <se3.hpp>
#include "tools.h"
#include <unordered_map>
#include <opencv2/opencv.hpp>

struct Frame
{
    Sophus::SE3f Twc;
    bool isPosed = false;
    int id = -1;
    double ts = 0.0;
};

class PostProcess
{
public:
    PostProcess();
    bool readPoses(const std::string& path);
    std::vector<std::pair<double, int>> readVideoLog(const std::string& path);
    std::unordered_map<int, std::vector<Eigen::Vector3f>> readLanePts(const std::string& path);
    Frame* getPosedFrm(int id);
    void setCamera(float* K, float* D);
    void setExtLidar2Cam(const Eigen::Vector3f& r, const Eigen::Vector3f& t);
    std::vector<Eigen::Vector2f> projPts2Img(const std::vector<Eigen::Vector3f>& pts, bool do_dist = true);
    std::vector<Eigen::Vector3f> cvt2CamCoord(const std::vector<Eigen::Vector3f>& pts);
    std::vector<Eigen::Vector3f> cvt2CamCoord(const std::vector<Eigen::Vector3f>& pts, const int id0, const int id1, const float r);
    std::vector<Eigen::Vector3f> cvt2CamCoord(const std::vector<Eigen::Vector3f>& pts, const double ts, const int id, bool do_filter = true, bool do_dist = true);
    std::vector<Eigen::Vector2f> drawIPMuvs(const std::vector<Eigen::Vector3f>& pts, float limx = 50, float limy = 50);
    bool loadMask(const std::string& path);
    void setMaskScale(float xs, float ys);

private:
    std::vector<Frame> frms_;
    std::unordered_map<int, Frame*> frm_map_;
    std::unordered_map<int, cv::Mat> mask_map_;
    Sophus::SE3f Tcl_;

    float fx, fy, cx, cy, k1, k2, p1, p2, k3;
    float x_scale_, y_scale_;
};

#endif