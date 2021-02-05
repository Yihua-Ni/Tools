#include<iostream>
#include<Eigen/Dense>
#include<Tools/Tools.h>

using namespace Eigen;

float my_random(void){
    float tmp = rand()%100;
    // cout<<"tmp:"<<tmp;
    return tmp/1000;
}


Matrix3d rotation_matrix(Eigen::Vector3d axis, float theta){
    axis = axis / sqrt(axis.transpose()*axis);
    float a = cos(theta/2);
    Eigen::Vector3d temp = -axis*sin(theta/2);
    float b,c,d;
    b = temp(0);
    c = temp(1);
    d = temp(2);
    Eigen::Matrix3d R;
    R << a*a+b*b-c*c-d*d, 2*(b*c-a*d), 2*(b*d+a*c),
        2*(b*c+a*d), a*a+c*c-b*b-d*d, 2*(c*d-a*b),
        2*(b*d-a*c), 2*(c*d+a*b), a*a+d*d-b*b-c*c;

    return R;
}


// nearest_neighbot
//平方根
float dist(const Eigen::Vector3d &pta, const Eigen::Vector3d &ptb){
    return sqrt((pta[0]-ptb[0])*(pta[0]-ptb[0]) + (pta[1]-ptb[1])*(pta[1]-ptb[1]) + (pta[2]-ptb[2])*(pta[2]-ptb[2]));
} 


NEIGHBOR nearest_neighbot(const Eigen::MatrixXd &src, const Eigen::MatrixXd &dst){
    int row_src = src.rows();     //
    int row_dst = dst.rows();
    Eigen::Vector3d vec_src;
    Eigen::Vector3d vec_dst;
    NEIGHBOR neigh;
    float min = 100;
    int index = 0;
    float dist_temp = 0;

    for(int ii=0; ii < row_src; ii++){
        vec_src = src.block<1,3>(ii,0).transpose();
        min = 100;
        index = 0;
        dist_temp = 0;
        for(int jj=0; jj < row_dst; jj++){
            vec_dst = dst.block<1,3>(jj,0).transpose();
            dist_temp = dist(vec_src,vec_dst);         
            if (dist_temp < min){
                min = dist_temp;
                index = jj;
            }
        }
        // cout << min << " " << index << endl;
        // neigh.distances[ii] = min;
        // neigh.indices[ii] = index;
        neigh.distances.push_back(min);
        neigh.indices.push_back(index);
    }
    return neigh;
}
