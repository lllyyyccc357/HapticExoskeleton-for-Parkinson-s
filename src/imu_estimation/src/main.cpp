#include "ros/ros.h"
#include "imu_estimation/IMU_thomasMag.h"
#include "common/data.h"
#include "common/gesture.h"

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
using namespace std;

const int N=2;


IMU_thomas IMU[N];
void doGyro(const common::data::ConstPtr& p_gyro);
void doAcc(const common::data::ConstPtr& p_acc);
void doMag(const common::data::ConstPtr& p_mag);
void GesPub(int imuNum,Eigen::Vector3d eulerAngles);
static ros::Publisher ges_pub;
static ros::Subscriber sub_gyro;
static ros::Subscriber sub_acc;
static ros::Subscriber sub_mag;
int main(int argc, char **argv) {

    for(int i=0;i<N;i++)
    {
        IMU[i].NUM=i;
    }
    // 初始化 ROS 节点
    ros::init(argc, argv, "IMU_subMes_pubGes");
    ros::NodeHandle nh;
    ges_pub=nh.advertise<common::gesture>("Gesture_pub",10);
    sub_gyro = nh.subscribe<common::data>("gyro_msg",100,doGyro);
    sub_acc = nh.subscribe<common::data>("acc_msg",100,doAcc);
    sub_mag = nh.subscribe<common::data>("mag_msg",100,doMag);
    
    // 循环，保持节点运行
    ros::spin();

    return 0;
}
void GesPub(int imuNum,Eigen::Vector3d euler_angles)
{
    common::gesture msg;
    msg.IMUnum=imuNum;
    msg.EulerAngleX=euler_angles[0];
    msg.EulerAngleY=euler_angles[1];
    msg.EulerAngleZ=euler_angles[2];
    ges_pub.publish(msg);
}
void doGyro(const common::data::ConstPtr& p_gyro)
{
    int number=p_gyro->num;
    if(~IMU[number].InitTrue){
        IMU[number].IMU_init();
    }
    if(IMU[number].InitTrue){
        IMU[number].Gyro << p_gyro->dataX,p_gyro->dataY,p_gyro->dataZ;
        IMU[number].Gyro_update();
        // ROS_INFO("yaw:%lf,pitch:%lf,roll:%lf",IMU[number].euler_angles[0],IMU[number].euler_angles[1],IMU[number].euler_angles[2]);
    }
    
}
void doAcc(const common::data::ConstPtr& p_acc)
{
    int number=p_acc->num;
    if(~IMU[number].InitTrue){
        IMU[number].Acc_init << p_acc->dataX,p_acc->dataY,p_acc->dataZ;
    }else{
        IMU[number].Acc << p_acc->dataX,p_acc->dataY,p_acc->dataZ;
        IMU[number].Acc_update();
        // GesPub(number,IMU[number].euler_angles);
        // ROS_INFO("yaw:%lf,pitch:%lf,roll:%lf",IMU[number].euler_angles[0],IMU[number].euler_angles[1],IMU[number].euler_angles[2]);
    }
}
void doMag(const common::data::ConstPtr& p_mag)
{
    int number=p_mag->num;
    if(~IMU[number].InitTrue){
        IMU[number].Mag_init << p_mag->dataX,p_mag->dataY,p_mag->dataZ;
    }else{
        IMU[number].Mag << p_mag->dataX,p_mag->dataY,p_mag->dataZ;
        IMU[number].Mag_update();
        GesPub(number,IMU[number].euler_angles);
        ROS_INFO("yaw:%lf,pitch:%lf,roll:%lf",IMU[number].euler_angles[0],IMU[number].euler_angles[1],IMU[number].euler_angles[2]);
    }
}
