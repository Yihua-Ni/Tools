#ifndef CAMBASE_H
#define CAMBASE_H

#include "types.h"

class CAMBASE
{
    public:

    virtual void undistAndNormlizePoint(const cv_pt2& in, cv_pt2& out) = 0;

    virtual void undistAndNormlizePoints(const std::vector<cv_pt2>& in_pts, std::vector<cv_pt2>& out_pts) = 0;

    virtual TF_FRONT focalLength() = 0;
};

#endif