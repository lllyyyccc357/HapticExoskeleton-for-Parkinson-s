#ifndef _IMU_THOMASMAG_H
#define _IMU_THOMASMAG_H

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/MatrixFunctions>
#include "common/data.h"
#include "imu_estimation/Quaterion_func_scalar4.h"
#include <cmath>
#include "ros/ros.h"
#include <iostream>

using namespace std;
double _g=9.81;
double stdAcc=0.0981;
double stdGyro=0.001*5;
double stdMag=0.02;
double sigma_acc=2.3;
double sigma_mag=1.7;

class IMU_thomas{
// this class is using the mag elimination method and multi-kernel kalman fliter
// The coordinate :NED
public:
    int NUM;// device name
    bool InitTrue;
    Eigen::Vector3d Acc;
    Eigen::Vector3d Mag;
    Eigen::Vector3d Acc_init;
    Eigen::Vector3d Mag_init;
    Eigen::Vector3d Gyro;
    Eigen::Vector3d sigma_y_acc;
    Eigen::Vector3d sigma_y_mag;
    Eigen::Vector4d x;
    Eigen::Vector3d euler_angles;
    Eigen::Vector4d qin;
    Eigen::Vector3d g_global;
    Eigen::Vector3d h_global;
    Eigen::Matrix<double, 4, 3> CSI;
    Eigen::Matrix4d Q;
    Eigen::Matrix3d SIGMA_g;
    Eigen::Matrix3d R_;
    Eigen::Matrix3d R_acc;
    Eigen::Matrix3d R_mag;
    
    Eigen::Matrix3d br_acc;
    Eigen::Matrix3d br_mag;
    Eigen::Matrix4d P;
    double dt;
    IMU_thomas(int n=-1,int frequency=400)
    {
        NUM=n;// the device num
        dt=1/frequency;
        x=Eigen::Vector4d::Zero(4,1);
        InitTrue=false;
        Acc_init.setZero();
        Mag_init.setZero();
    }
    bool IMU_init()
    {
        if(Acc_init.isZero()|| Mag_init.isZero())
        {
            return false;
        }
        else{
            Eigen::Vector3d r_down=Acc_init;
            Eigen::Vector3d r_east=Acc_init.cross(Mag_init);
            Eigen::Vector3d r_north=r_east.cross(r_down);
            r_down=r_down/r_down.norm();
            r_east=r_east/r_east.norm();
            r_north=r_north/r_north.norm();
            R_<<r_north,r_east,r_down;
            Eigen::Quaterniond Q_(R_);  
            qin<<Q_.x(),Q_.y(),Q_.z(),Q_.w();
            x=qin;
            Eigen::Vector3d L=R_.transpose()*Mag_init;
            h_global<<std::sqrt(L[0]*L[0]+L[1]*L[1]),0,L[2];
            SIGMA_g=stdGyro*stdGyro*Eigen::Matrix3d::Identity();
            dt=1/400;
            CSI<<qin[3],-qin[2],qin[1],
                qin[2],qin[3],-qin[0],
                -qin[1],qin[0],qin[3],
                -qin[0],-qin[1],-qin[2];
            Q=(dt/2)*(dt/2)*CSI*SIGMA_g*CSI.transpose();
            P=Q;
            sigma_y_acc<<sigma_acc,sigma_acc,sigma_acc;
            sigma_y_mag<<sigma_mag,sigma_mag,sigma_mag;
            R_acc=stdAcc*stdAcc*Eigen::Matrix3d::Identity();
            R_mag=stdMag*stdMag*Eigen::Matrix3d::Identity();
            br_acc=stdAcc*Eigen::Matrix3d::Identity();
            br_mag=stdMag*Eigen::Matrix3d::Identity();
            h_global<<1,0,0;
            g_global<<0,0,_g;
            InitTrue=true;
            return true;
            }
    }
    void Gyro_update()
    {
        Eigen::Matrix4d omega;
        omega << 0,Gyro[2],-Gyro[1],Gyro[0],
                -Gyro[2],0,Gyro[0],Gyro[1],
                Gyro[1],-Gyro[0],0,Gyro[2],
                -Gyro[0],-Gyro[1],-Gyro[2],0;
        omega=0.5*omega;
                    CSI<<qin[3],-qin[2],qin[1],
                qin[2],qin[3],-qin[0],
                -qin[1],qin[0],qin[3],
                -qin[0],-qin[1],-qin[2];
        Q=(dt/2)*(dt/2)*CSI*SIGMA_g*CSI.transpose();
        Eigen::Matrix4d F = (omega * dt).exp();
        x=F*x;
        P=F*P*F.transpose()+Q;
        x2Euler();
    }
    void Acc_update()
    {
        double q1=x[0];
        double q2=x[1];
        double q3=x[2];
        double q4=x[3];
        Eigen::Matrix<double, 3, 4> H_acc;
        H_acc<<q3,-q4,q1,-q2,
                q4,q3,q2,q1,     
                -q1,-q2,q3,q4;
        H_acc=2*_g*H_acc;
        Eigen::Matrix3d C;
         C << q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4,  2 * (q1 * q2 + q3 * q4),       2 * (q1 * q3 - q2 * q4),
         2 * (q1 * q2 - q3 * q4),               -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4, 2 * (q2 * q3 + q1 * q4),
         2 * (q1 * q3 + q2 * q4),                2 * (q2 * q3 - q1 * q4),      -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4;
        Eigen::Vector3d z_predict_acc=C*g_global;
        Eigen::Vector3d z_acc=Acc;
        Eigen::Vector3d ze_acc=z_acc-z_predict_acc;
        Eigen::Vector3d er_acc = br_acc.inverse() * ze_acc;
        int cnt=6;
        int num=cnt;
        Eigen::Vector4d x_tlast(0,0,0,0);
        Eigen::Vector4d x_t(0,0,0,0);
        Eigen::Vector3d diay_acc;
        Eigen::Matrix3d Cy_acc, R_1_acc;
        Eigen::MatrixXd K_1_acc(4, 3);
        while(num>0)
        {
            if(num==cnt){
                x_tlast=x;
            }else{
                x_tlast=x_t;
            }
            q1=x_tlast[0];
            q2=x_tlast[1];
            q3=x_tlast[2];
            q4=x_tlast[3];
            C << q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4,  2 * (q1 * q2 + q3 * q4),2 * (q1 * q3 - q2 * q4),
         2 * (q1 * q2 - q3 * q4), -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4, 2 * (q2 * q3 + q1 * q4),
         2 * (q1 * q3 + q2 * q4),  2 * (q2 * q3 - q1 * q4), -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4;
            z_predict_acc=C*g_global;
            ze_acc=z_acc-z_predict_acc;
            er_acc = br_acc.inverse() * ze_acc;
            diay_acc = (-er_acc.array().square() / sigma_y_acc.array()).exp();
            for(int k=0;k<3;k++)
            {
                if(diay_acc(k)<1e-8)
                diay_acc(k)=1e-8;
            }
            
            Cy_acc = diay_acc.asDiagonal();
            R_1_acc = br_acc * Cy_acc.inverse() * br_acc.transpose();
            K_1_acc = P * H_acc.transpose() * (H_acc * P * H_acc.transpose() + R_1_acc).inverse();
            x_t = x + K_1_acc * (z_acc - z_predict_acc);
            num=num-1;
            double thresh = (x_t - x_tlast).norm() / (x_tlast.norm() + 1e-3);
            if (thresh < 1e-13) {
                break;
            }
        }
        x_t /= x_t.norm();
        x=x_t;
        P= (Eigen::Matrix4d::Identity() - K_1_acc * H_acc) * P* (Eigen::Matrix4d::Identity() - K_1_acc * H_acc).transpose() + K_1_acc * R_1_acc * K_1_acc.transpose();
        x2Euler();
    }
    void Mag_update()
    {
        double q1=x[0];
        double q2=x[1];
        double q3=x[2];
        double q4=x[3];
        Eigen::Matrix<double, 3, 4> H_mag;

        H_mag<< q1,-q2,-q3,q4,
                q2,q1,-q4,-q3,
                q3,q4,q1,q2;
        H_mag=2*H_mag;
        Eigen::Matrix3d C;
        C << q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4,  2 * (q1 * q2 + q3 * q4),       2 * (q1 * q3 - q2 * q4),
        2 * (q1 * q2 - q3 * q4),               -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4, 2 * (q2 * q3 + q1 * q4),
        2 * (q1 * q3 + q2 * q4),                2 * (q2 * q3 - q1 * q4),      -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4;
        Eigen::Vector4d q_e2s_gyr=x;
        q_e2s_gyr=q_e2s_gyr/q_e2s_gyr.norm();
        Eigen::Vector4d unitVec_g(0,0,1,0);
        Eigen::Vector4d q_s2e_gyr=QuaterionConj(q_e2s_gyr);
        Eigen::Vector4d q_s_acc=QuaterionProd(q_s2e_gyr,QuaterionProd(unitVec_g,q_e2s_gyr));
        Eigen::Vector3d r_s_acc(q_s_acc[0],q_s_acc[1],q_s_acc[2]);
        Eigen::Vector3d m_hat_s=Mag-Mag.dot(r_s_acc)*r_s_acc;
        m_hat_s/=m_hat_s.norm();
        
        Eigen::Vector3d z_predict_mag=C*h_global;
        Eigen::Vector3d z_mag=m_hat_s;
        Eigen::Vector3d ze_mag=z_mag-z_predict_mag;
        Eigen::Vector3d er_mag = br_mag.inverse() * ze_mag;
        int cnt=5;
        int num=cnt;
        Eigen::Vector4d x_tlast(0,0,0,0);
        Eigen::Vector4d x_t(0,0,0,0);
        Eigen::Vector3d diay_mag;
        Eigen::Matrix3d Cy_mag, R_1_mag;
        Eigen::MatrixXd K_1_mag(4, 3);
        while(num>0)
        {
            if(num==cnt){
                x_tlast=x;
            }else{
                x_tlast=x_t;
            }
            q1=x_tlast[0];
            q2=x_tlast[1];
            q3=x_tlast[2];
            q4=x_tlast[3];
            H_mag<< q1,-q2,-q3,q4,
                q2,q1,-q4,-q3,
                q3,q4,q1,q2;
            H_mag=2*H_mag;
            C << q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4,  2 * (q1 * q2 + q3 * q4),2 * (q1 * q3 - q2 * q4),
         2 * (q1 * q2 - q3 * q4), -q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4, 2 * (q2 * q3 + q1 * q4),
         2 * (q1 * q3 + q2 * q4),  2 * (q2 * q3 - q1 * q4), -q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4;
            q_e2s_gyr=x_tlast;
            q_e2s_gyr=q_e2s_gyr/q_e2s_gyr.norm();
            q_s2e_gyr=QuaterionConj(q_e2s_gyr);
            q_s_acc=QuaterionProd(q_s2e_gyr,QuaterionProd(unitVec_g,q_e2s_gyr));
            r_s_acc<<q_s_acc[0],q_s_acc[1],q_s_acc[2];
            m_hat_s=Mag-Mag.dot(r_s_acc)*r_s_acc;
            m_hat_s/=m_hat_s.norm();
            z_mag=m_hat_s;
            z_predict_mag=C*h_global;
            ze_mag=z_mag-z_predict_mag;
            er_mag = br_mag.inverse() * ze_mag;
            diay_mag = (-er_mag.array().square() / sigma_y_mag.array()).exp();
            for(int k=0;k<3;k++)
            {
                if(diay_mag(k)<1e-16)
                diay_mag(k)=1e-16;
            }
            
            Cy_mag = diay_mag.asDiagonal();
            R_1_mag = br_mag * Cy_mag.inverse() * br_mag.transpose();
            K_1_mag = P * H_mag.transpose() * (H_mag * P * H_mag.transpose() + R_1_mag).inverse();
            x_t = x + K_1_mag * (z_mag - z_predict_mag);
            num=num-1;
            double thresh = (x_t - x_tlast).norm() / (x_tlast.norm() + 1e-3);
            if (thresh < 1e-16) {
                break;
            }
        }
        x_t /= x_t.norm();
        x=x_t;
        P= (Eigen::Matrix4d::Identity() - K_1_mag * H_mag) * P* (Eigen::Matrix4d::Identity() - K_1_mag * H_mag).transpose() + K_1_mag * R_1_mag * K_1_mag.transpose();
        x2Euler();
    }
    void x2Euler()
    {
         Eigen::Quaterniond q(x[3], x[0], x[1], x[2]); // w, x, y, z

        // 归一化四元数（确保其表示有效的旋转）
        q.normalize();

            // roll (x-axis rotation)
        double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
        double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
        euler_angles(2) = std::atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
        if (std::abs(sinp) >= 1)
            euler_angles(1) = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
        else
            euler_angles(1) = std::asin(sinp);

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
        double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
        euler_angles(0) = std::atan2(siny_cosp, cosy_cosp);
        euler_angles*=(180/3.14);
    }
};



#endif