#ifndef ICP_H
#define ICP_H

#include <Eigen/Dense>
#include "ICPBase.h"

#define MAX_ITERATION 10
#define TOLERANCE 0.001
#define K 1.2

using namespace std;
using namespace Eigen;


typedef struct{
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;

class ICP: public ICPBase
{
public:

    // ICP();
    MatrixXd icp(const MatrixXd pts1, const MatrixXd pts2, int max_iteration=MAX_ITERATION, float tolerance=TOLERANCE, float k = K);
    Matrix3d rotation_matrix(Eigen::Vector3d axis, float theta);
    float my_random(void);
    Matrix4d SVDsolve(const MatrixXd &pts1, const MatrixXd &pts2);
   
private:
    NEIGHBOR nearest_neighbot(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);
    float dist(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb);

};


#endif
