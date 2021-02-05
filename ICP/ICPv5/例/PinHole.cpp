#include "PinHole.h"


PINHOLE::PINHOLE(const cv::Mat& K, const cv::Mat& D, TF scale)
{
    // K *= scale;
    K.copyTo(K_);
	K_ *= scale;
    D.copyTo(D_);

	fx_ = K_.at<TF_FRONT>(0,0);
	cx_ = K_.at<TF_FRONT>(0,2);
	fy_ = K_.at<TF_FRONT>(1,1);
	cy_ = K_.at<TF_FRONT>(1,2);
	// std::cout << fx_<< ", " << fy_ << ", " << cx_ << ", "<< cy_ << std::endl;
}

void PINHOLE::undistAndNormlizePoint(const cv_pt2& in, cv_pt2& out)
{
	std::vector<cv_pt2> va;
	va.push_back(in);
	std::vector<cv_pt2> vb;
	cv::undistortPoints(va, vb, K_, D_);
	out.x = vb[0].x;
	out.y = vb[0].y;
}

void PINHOLE::undistAndNormlizePoints(const std::vector<cv_pt2>& in_pts, std::vector<cv_pt2>& out_pts)
{
	cv::undistortPoints(in_pts, out_pts, K_, D_);
}

TF_FRONT PINHOLE::focalLength()
{
	return fx_;
}

TF_FRONT PINHOLE::fx()
{
	return fx_;
}


TF_FRONT PINHOLE::cx()
{
	return cx_;
}


TF_FRONT PINHOLE::fy()
{
	return fy_;
}


TF_FRONT PINHOLE::cy()
{
	return cy_;
}