#include<vector>
#include<Eigen/Dense>

using namespace Eigen;

typedef struct{
    std::vector<float> distances;
    std::vector<int> indices;
} NEIGHBOR;

float my_random(void);

Matrix3d rotation_matrix(Eigen::Vector3d axis, float theta);

NEIGHBOR nearest_neighbot(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst);