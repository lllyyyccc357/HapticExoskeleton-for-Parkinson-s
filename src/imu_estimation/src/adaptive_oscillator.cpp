#include "common/gesture.h"
#include "ros/ros.h"
#include "adaptive_osc/AdaptiveOsc.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
using namespace std;
const int N=2;
AdaptiveOscillator AO[N];
int GesAngleUsing;
void doGes(const common::gesture::ConstPtr& p_ges);
int main(int argc, char **argv)
{
    for(int i=0;i<N;i++)
    {
        AO[i].NUM=i;
    }
    // 初始化 ROS 节点
    ros::init(argc, argv,"IMU_subGes_pubPhase");
    for(int i=0;i<N;i++)
    {
        AO[i].NUM=i;
        ROS_INFO("initial:output:%lf,phase:%lf,Freq:%lf",AO[i].getOutput(),AO[i].getPhase(),AO[i].getFrequency());
    }
    ros::NodeHandle nh;
    ros::param::get("euler_angle_adaptive_osc_using",GesAngleUsing);
    // ROS_INFO("%d",GesAngleUsing);
    ros::Subscriber sub_Ges = nh.subscribe<common::gesture>("Gesture_pub",100,doGes);

    ros::spin();
    return 0;
}
void doGes(const common::gesture::ConstPtr& p_ges)
{
    int number=p_ges->IMUnum;
    // ROS_INFO("start estimate phase");
    double signal;
    if(GesAngleUsing=0){
        signal=p_ges->EulerAngleX;
    }else if (GesAngleUsing=1){
        signal=p_ges->EulerAngleY;
    }else if (GesAngleUsing=2){
        signal=p_ges->EulerAngleZ;
    }
    AO[number].update(signal);
    ROS_INFO("num:%d:output:%lf,phase:%lf,Freq:%lf",number,AO[number].getOutput(),AO[number].getPhase(),AO[number].getFrequency());
}