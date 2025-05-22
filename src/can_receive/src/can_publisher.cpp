<<<<<<< HEAD
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息

#include <sstream>

#include "common/data.h"
int Gyro_id[2]={0x32,0x3c};
int Mag_id[2]={0x41,0x4b};
int Acc_id[2]={0x34,0x3e};
int Can2Msg(can_frame f,common::data& d);
double int16_2_double(int8_t t1,int8_t t2);
int main(int argc, char  *argv[])
{
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));
    
    system("sudo ip link set can0 type can bitrate 500000");
    system("sudo ifconfig can0 up");
    
    //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
        return 1;
    }
    
    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }

    //3.Bind the socket to can0
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
        return 1;
    }
    
    //4.Define receive rules
    struct can_filter rfilter[6];
    rfilter[0].can_id = Gyro_id[0];
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = Gyro_id[1];
    rfilter[1].can_mask = CAN_SFF_MASK;

    rfilter[2].can_id = Mag_id[0];
    rfilter[2].can_mask = CAN_SFF_MASK;
    rfilter[3].can_id = Mag_id[1];
    rfilter[3].can_mask = CAN_SFF_MASK;

    rfilter[4].can_id = Acc_id[0];
    rfilter[4].can_mask = CAN_SFF_MASK;
    rfilter[5].can_id = Acc_id[1];
    rfilter[5].can_mask = CAN_SFF_MASK;
    
    
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"can_publisher");
    //3.实例化 ROS 句柄deviceNum
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub_gyro = nh.advertise<common::data>("gyro_msg",100);
    ros::Publisher pub_acc = nh.advertise<common::data>("acc_msg",100);
    ros::Publisher pub_mag = nh.advertise<common::data>("mag_msg",100);
    common::data _data;
    _data.num=0;
    _data.name=0;
    _data.dataX=0;
    _data.dataY=0;
    _data.dataZ=0;
    //5.Receive data and exit
    while(1) {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        nbytes = read(s, &frame, sizeof(frame));
        if(nbytes > 0) {
            int flag=Can2Msg(frame,_data);
            if(flag==1){
                pub_gyro.publish(_data);
            }else if(flag==2){
                pub_acc.publish(_data);
            }else if(flag==3){
                pub_mag.publish(_data);
            }
            if(flag==3)
            ROS_INFO("The msg : name: %d num: %d X: %f Y %f Z %f",_data.name,_data.num,_data.dataX,_data.dataY,_data.dataZ);
        }
    }
    
    //6.Close the socket and can0
    close(s);
    system("sudo ifconfig can0 down");
    
    return 0;
}
double int16_2_double(int8_t t1,int8_t t2)
{
    uint8_t high = t1; // 高8位（示例值）
    uint8_t low = t2;  // 低8位（示例值）
    // 将 uint16_t 转换为 double
    double result = 0;
    // 将高8位和低8位组合成一个 uint16_t
    int combined = (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    if (combined>=32678){
        result=combined-65536;
    }
    else{
        result=combined;
    }

    
    return result;
}
int Can2Msg(can_frame f,common::data& d)
{
    int deviceNum=2;
    double gyro_res=0.001953;
    double acc_res=0.003906;
    double mag_res=0.00097656;
    for(int i=0;i<deviceNum;i++)
    {
        if(f.can_id==Gyro_id[i]){
            d.num=i;
            d.name=1;
            d.dataX=int16_2_double(f.data[0],f.data[1])*gyro_res;
            d.dataY=int16_2_double(f.data[2],f.data[3])*gyro_res;
            d.dataZ=int16_2_double(f.data[4],f.data[5])*gyro_res;
        }else if(f.can_id==Acc_id[i]){
            d.num=i;
            d.name=2;
            d.dataX=-int16_2_double(f.data[0],f.data[1])*acc_res;
            d.dataY=-int16_2_double(f.data[2],f.data[3])*acc_res;
            d.dataZ=-int16_2_double(f.data[4],f.data[5])*acc_res;
        }else if(f.can_id==Mag_id[i]){
            d.num=i;
            d.name=3;
            d.dataX=int16_2_double(f.data[0],f.data[1])*mag_res;
            d.dataY=int16_2_double(f.data[2],f.data[3])*mag_res;
            d.dataZ=int16_2_double(f.data[4],f.data[5])*mag_res;
        }
    }
    return d.name;
=======
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
// 1.包含头文件 
#include "ros/ros.h"
#include "std_msgs/String.h" //普通文本类型的消息

#include <sstream>

#include "common/data.h"
int Gyro_id[2]={0x32,0x3c};
int Mag_id[2]={0x41,0x4b};
int Acc_id[2]={0x34,0x3e};
int Can2Msg(can_frame f,common::data& d);
double int16_2_double(int8_t t1,int8_t t2);
int main(int argc, char  *argv[])
{
    int ret;
    int s, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    
    memset(&frame, 0, sizeof(struct can_frame));
    system("sudo ifconfig can0 down");
    
    system("sudo ip link set can0 type can bitrate 1000000");
    
    system("sudo ifconfig can0 up");
    
    //1.Create socket
    s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) {
        perror("socket PF_CAN failed");
        return 1;
    }
    
    //2.Specify can0 device
    strcpy(ifr.ifr_name, "can0");
    ret = ioctl(s, SIOCGIFINDEX, &ifr);
    if (ret < 0) {
        perror("ioctl failed");
        return 1;
    }

    //3.Bind the socket to can0
    addr.can_family = PF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    ret = bind(s, (struct sockaddr *)&addr, sizeof(addr));
    if (ret < 0) {
        perror("bind failed");
        return 1;
    }
    
    //4.Define receive rules
    struct can_filter rfilter[6];
    rfilter[0].can_id = Gyro_id[0];
    rfilter[0].can_mask = CAN_SFF_MASK;
    rfilter[1].can_id = Gyro_id[1];
    rfilter[1].can_mask = CAN_SFF_MASK;

    rfilter[2].can_id = Mag_id[0];
    rfilter[2].can_mask = CAN_SFF_MASK;
    rfilter[3].can_id = Mag_id[1];
    rfilter[3].can_mask = CAN_SFF_MASK;

    rfilter[4].can_id = Acc_id[0];
    rfilter[4].can_mask = CAN_SFF_MASK;
    rfilter[5].can_id = Acc_id[1];
    rfilter[5].can_mask = CAN_SFF_MASK;
    
    
    //设置编码
    setlocale(LC_ALL,"");

    //2.初始化 ROS 节点:命名(唯一)
    // 参数1和参数2 后期为节点传值会使用
    // 参数3 是节点名称，是一个标识符，需要保证运行后，在 ROS 网络拓扑中唯一
    ros::init(argc,argv,"can_publisher");
    //3.实例化 ROS 句柄deviceNum
    ros::NodeHandle nh;//该类封装了 ROS 中的一些常用功能

    //4.实例化 发布者 对象
    //泛型: 发布的消息类型
    //参数1: 要发布到的话题
    //参数2: 队列中最大保存的消息数，超出此阀值时，先进的先销毁(时间早的先销毁)
    ros::Publisher pub_gyro = nh.advertise<common::data>("gyro_msg",100);
    ros::Publisher pub_acc = nh.advertise<common::data>("acc_msg",100);
    ros::Publisher pub_mag = nh.advertise<common::data>("mag_msg",100);
    common::data _data;
    _data.num=0;
    _data.name=0;
    _data.dataX=0;
    _data.dataY=0;
    _data.dataZ=0;
    //5.Receive data and exit
    while(1) {
        setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));
        nbytes = read(s, &frame, sizeof(frame));
        if(nbytes > 0) {
            int flag=Can2Msg(frame,_data);
            if(flag==1){
                pub_gyro.publish(_data);
            }else if(flag==2){
                pub_acc.publish(_data);
            }else if(flag==3){
                pub_mag.publish(_data);
            }
            // if(flag==3)
            // ROS_INFO("The msg : name: %d num: %d X: %f Y %f Z %f",_data.name,_data.num,_data.dataX,_data.dataY,_data.dataZ);
        }
    }
    
    //6.Close the socket and can0
    close(s);
    system("sudo ifconfig can0 down");
    
    return 0;
}
double int16_2_double(int8_t t1,int8_t t2)
{
    uint8_t high = t1; // 高8位（示例值）
    uint8_t low = t2;  // 低8位（示例值）
    // 将 uint16_t 转换为 double
    double result = 0;
    // 将高8位和低8位组合成一个 uint16_t
    int combined = (static_cast<uint16_t>(high) << 8) | static_cast<uint16_t>(low);
    if (combined>=32678){
        result=combined-65536;
    }
    else{
        result=combined;
    }

    
    return result;
}
int Can2Msg(can_frame f,common::data& d)
{
    int deviceNum=2;
    double gyro_res=0.001953;
    double acc_res=0.003906;
    double mag_res=0.00097656;
    for(int i=0;i<deviceNum;i++)
    {
        if(f.can_id==Gyro_id[i]){
            d.num=i;
            d.name=1;
            d.dataX=int16_2_double(f.data[0],f.data[1])*gyro_res;
            d.dataY=int16_2_double(f.data[2],f.data[3])*gyro_res;
            d.dataZ=int16_2_double(f.data[4],f.data[5])*gyro_res;
        }else if(f.can_id==Acc_id[i]){
            d.num=i;
            d.name=2;
            d.dataX=-int16_2_double(f.data[0],f.data[1])*acc_res;
            d.dataY=-int16_2_double(f.data[2],f.data[3])*acc_res;
            d.dataZ=-int16_2_double(f.data[4],f.data[5])*acc_res;
        }else if(f.can_id==Mag_id[i]){
            d.num=i;
            d.name=3;
            d.dataX=int16_2_double(f.data[0],f.data[1])*mag_res;
            d.dataY=int16_2_double(f.data[2],f.data[3])*mag_res;
            d.dataZ=int16_2_double(f.data[4],f.data[5])*mag_res;
        }
    }
    return d.name;
>>>>>>> 0af27662387a1404fe8b6f6909ba863f190f46ef
}