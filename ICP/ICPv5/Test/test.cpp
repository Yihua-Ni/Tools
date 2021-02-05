#include<iostream>
#include<Eigen/Dense>
#include<vector>
#include <memory>
#include"ICP/ICP.h"

using namespace std;
using namespace Eigen;

int main()
{
  //std::shared_ptr<ICPBase> ptr;
  //ptr = std::make_shared<ICP>();


  ICP A;
  float rotation = 5;
  float noise_sigmod = 0.01;
  int Tk = 1;
  int N = 10;
  int m = 3;

  MatrixXd noise =  MatrixXd::Random(N, m) * noise_sigmod ;
  // cout<<"noise:"<<noise<<endl;

  MatrixXd pts1 = MatrixXd::Random(N, m)*10;
  // cout<<"pts1:"<<pts1<<endl;
  MatrixXd pts2 = pts1;
  // MatrixXd t = MatrixXd::Random(1, m) * Tk;   //(1,3)
  RowVector3d t = Vector3d::Random() * Tk;
  // Matrix3d R = Matrix3d::Identity();      //(3,3)
  Matrix3d R = A.rotation_matrix(Vector3d::Random(), A.my_random()*rotation);
  cout<<"t:"<<t<<endl<<"R:"<<R<<endl;
  pts2 = pts2 + noise;                   //(10,3)
  // cout<<"pts2:"<<pts2<<endl;
  pts2 = (R * pts2.transpose()).transpose() ;   
  pts2.rowwise() += t ;      
  pts2 = pts2 + noise;
  pts1.conservativeResize(N+2, m);
  pts2.conservativeResize(N+1, m);
  pts1.row(10)<< 3.5, 4.6, 8.1;
  pts1.row(11)<< 5.2, 1.7, 5.7;
  pts2.row(10)<< 4.1, 5.9, 9.9;
  cout<<"pts1:"<<pts1<<endl<<"pts2:"<<pts2<<endl;

//   SVDsolve(pts1, pts2);
  MatrixXd T = A.icp(pts1, pts2);
  cout << "T:" << T << endl;

  return 0;
}
