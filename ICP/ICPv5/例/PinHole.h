#ifndef PINHOLE_H
#define PINHOLE_H

#include "CamBase.h"

class PINHOLE : public CAMBASE
{
    public :

    PINHOLE() = delete;

    PINHOLE(const cv::Mat& K, const cv::Mat& D, TF scale);

    void undistAndNormlizePoint(const cv_pt2& in, cv_pt2& out);

    void undistAndNormlizePoints(const std::vector<cv_pt2>& in_pts, std::vector<cv_pt2>& out_pts);

    TF_FRONT focalLength();

    TF_FRONT fx();

    TF_FRONT fy();

    TF_FRONT cx();

    TF_FRONT cy();

    private:

    private:
    cv::Mat K_;
    cv::Mat D_;

    TF_FRONT fx_, fy_, cx_, cy_;
};

#endif