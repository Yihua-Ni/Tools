#include <iostream>
#include <Eigen/Dense>
#include "Tools/Tools.h"
#include "ICP.h"


using namespace std;
using namespace Eigen;

// Matrix4d SVDsolve(const MatrixXd &pts1, const MatrixXd &pts2)
Matrix4d SVDsolve(const MatrixXd &pts1, const MatrixXd &pts2)
{
  Matrix4d T = MatrixXd::Identity(4,4);
  Vector3d p1(0,0,0);
  Vector3d p2(0,0,0);
  int N = pts1.rows();
  int m = pts1.cols();
  // cout<<"N:" << N << "   m:"<< m << endl;
  // cout<<"pts1:"<<pts1<< endl << "pts2:"<<pts2<<endl;
  MatrixXd q1(N, m);
  MatrixXd q2(N, m);

  for (int i=0; i<N; i++){
      p1 = p1 + pts1.block<1,3>(i,0).transpose();
      p2 = p2 + pts2.block<1,3>(i,0).transpose();
  };
  p1 = p1/N;
  p2 = p2/N;
  // cout<<"p1:"<< p1 <<endl<< "p2:"<< p2 << endl;

  for (int i=0; i<N; i++){
      q1.block<1, 3>(i, 0) = pts1.block<1, 3>(i, 0) - p1.transpose();  //(starting at)  <block of size>
      q2.block<1, 3>(i, 0) = pts2.block<1, 3>(i, 0) - p2.transpose();
  };
  // cout<<"q1:"<< q1 <<endl<< "q2:"<< q2 << endl;

  MatrixXd W = q1.transpose() * q2;
  // cout<<"W:"<<W<<endl;

  JacobiSVD<MatrixXd> svd(W, ComputeThinU | ComputeThinV);
    MatrixXd U = svd.matrixU();
    MatrixXd V = svd.matrixV();
    MatrixXd Vt = V.transpose();
  // cout<<"U:"<<U<<endl<<"V:"<<V<<endl<<"Vt:"<<Vt<<endl;

  MatrixXd R = Vt.transpose() * U.transpose();
  // cout<<"R:"<<R<<endl;

//   Matrix3d xx(3,3);
//   xx<<1,1,1,
//       1,1,1,
//       1,1,1 ;
//   xx.block<1,3>(2,0)*= -1;
//   cout<<xx<<endl;

  if(R.determinant() < 0){
 //R行列式<0
    Vt.block<1, 3>(2, 0) *= -1;       //第三行乘以-1
    R = Vt.transpose() * U.transpose();
    // cout<<"Vt:"<<Vt<<endl;
  };
  // cout<<"R:"<<R<<endl;
  Vector3d t = p2 - R*p1;      //(3,1) - (3,3)(3,1) = (3,1)
  // cout<<"t:"<<t<<endl;

  T.block<3,3>(0,0) = R ;
  T.block<3,1>(0,3) = t ;
  // cout<<"T"<<T<<endl;
  return T;
};


MatrixXd ICP(const MatrixXd pts1, const MatrixXd pts2, int max_iteration, float tolerance, float k ){
  int m = pts1.cols();
  int N1 = pts1.rows();
  int N2 = pts2.rows();
  MatrixXd src = MatrixXd::Ones(m+1, N1);    //(4, 12)
  MatrixXd dst = MatrixXd::Ones(m+1, N2);    //(4, 12)
  // cout<<"src:"<<src<<endl<<"dst:"<<dst<<endl;
  src.block(0,0,m,N1) = pts1.transpose();       
  dst.block(0,0,m,N2) = pts2.transpose();
  // cout<<"src:"<<src<<endl<<"dst:"<<dst<<endl;

  float prev_error = 0;
  
  for(int i =0; i<max_iteration; i++){
    //   NEIGHBOR nearest_neighbot(src.block(0,0,m,N1), dst.block(0,0,m,N2));
    //   NEIGHBOR nighbor = nearest_neighbot(src.block(0,0,m,N1).transpose(), dst.block(0,0,m,N2).transpose());
      NEIGHBOR nighbor = nearest_neighbot(src.block(0,0,m,N1).transpose(), dst.block(0,0,m,N2).transpose());
      std::vector<float> distance = nighbor.distances;
      std::vector<int> indices = nighbor.indices;
      float mean_error=0;
      for(int i = 0; i<distance.size(); i++){
          mean_error = mean_error + distance[i];
          // cout<<"distance:"<<distance[i]<<endl;
      };
      mean_error = mean_error/distance.size();
      cout<<"mean_error:"<<mean_error<<endl;
      // for(int i = 0; i<indices.size(); i++){
      // cout<<"indices:"<<indices[i]<<endl;};

      int j = 0 ;
      for (int i = 0; i<N1 ; i++){
         if(distance[i]<=mean_error*k){
             j = j+1;
         };
      };
      cout<<"成功匹配:"<<j<<"对点"<<endl;


      MatrixXd pts11 = MatrixXd::Ones(j, 3);
      MatrixXd pts22 = MatrixXd::Ones(j, 3);
      // cout<<"pts11:"<<pts11<<endl<<"pts22:"<<pts22<<endl;

      // pts11.block<1,3>(0,0) = src.block<3,1>(0, 0);
      // pts11.block<1,3>(1,0) = src.block<3,1>(0, 1);

      int jj = 0;
      for (int i =0; i<N1; i++){
          if(distance[i]<= mean_error*k){
              pts11.block<1,3>(jj,0) = src.block<3,1>(0, i);
              pts22.block<1,3>(jj,0) = dst.block<3,1>(0, indices[i]);
              jj = jj +1;
                 }
          }
      cout<<"pts11:"<<pts11<<endl<<"pts22:"<<pts22<<endl;

      MatrixXd T = SVDsolve(pts11, pts22);
      cout << "T:" << T << endl;
      src = T*src;

      if (fabs(prev_error - mean_error) < tolerance){
        break;
      }
      prev_error = mean_error;

      }
  MatrixXd T = SVDsolve(pts1, src.block(0,0,m,N1).transpose());

  return T;

};






