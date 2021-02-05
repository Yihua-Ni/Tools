#ifndef ICPBase_H
#define ICPBase_H

#include<Eigen/Dense>

class ICPBase
{
public:

    virtual Eigen::Matrix4d SVDsolve(const Eigen::MatrixXd &pts1, const Eigen::MatrixXd &pts2) = 0;

};


#endif