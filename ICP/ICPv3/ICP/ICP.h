#include<Eigen/Dense>

using namespace Eigen;


Matrix4d SVDsolve(const MatrixXd &pts1, const MatrixXd &pts2);
MatrixXd ICP(const MatrixXd pts1, const MatrixXd pts2, int max_iteration=10, float tolerance=0.001, float k = 1.2);


// #ifndef ICP_H
// #define ICP_H

// #include<Eigen/Dense>

// #define MAX_ITERATION 10
// #define TOLERANCE 0.001
// #define K 1.2

// using namespace std;
// using namespace Eigen;


// class ICP
// {
// public:
//     ICP();
//     MatrixXd ICP(const MatrixXd pts1, const MatrixXd pts2, int max_iteration=MAX_ITERATION, float tolerance=TOLERANCE, float k = K);
// private:
//     Matrix4d SVDsolve(const MatrixXd &pts1, const MatrixXd &pts2);
// };


// #endif