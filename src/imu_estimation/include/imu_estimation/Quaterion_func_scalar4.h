#ifndef _QUATERION_FUNC_SCALAR4_H
#define _QUATERION_FUNC_SCALAR4_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <cmath>
using namespace std;

Eigen::Vector4d QuaterionAdd(Eigen::Vector4d q,Eigen::Vector4d r)
{
    Eigen::Vector4d qr;
    for(int i=0;i<4;i++){
        qr[i]=q[i]+qr[i];
    }
    return qr;
}

Eigen::Vector4d QuaterionProd(Eigen::Vector4d q,Eigen::Vector4d r)
{
    Eigen::Vector4d qr;
    qr[0] = q[1] * r[2] - q[2] * r[1] + q[0] * r[3] + q[3] * r[0];
    qr[1] = q[2] * r[0] - q[0] * r[2] + q[1] * r[3] + q[3] * r[1];
    qr[2] = q[0] * r[1] - q[1] * r[0] + q[2] * r[3] + q[3] * r[2];
    qr[3] = q[3] * r[3] - q[0] * r[0] - q[1] * r[1] - q[2] * r[2];
    return qr;
}

double QuaterionNorm(Eigen::Vector4d q)
{
    double ans=std::sqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    return ans;
}

Eigen::Vector4d QuaterionConj(Eigen::Vector4d q)
{
    Eigen::Vector4d qconj;
    qconj=q;
    qconj[3]=-qconj[3];
    return qconj;
}

Eigen::Vector4d QuaterionInv(Eigen::Vector4d q)
{
    Eigen::Vector4d qinv;
    Eigen::Vector4d qconj=QuaterionConj(q);
    double qNorm=QuaterionNorm(q);
    qinv=qconj;
    for(int i=0;i<4;i++){
        qinv[i]=qinv[i]/qNorm;
    }
    return qinv;
}

#endif