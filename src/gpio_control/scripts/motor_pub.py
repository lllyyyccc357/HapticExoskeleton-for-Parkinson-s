#! /usr/bin/env python
"""
    发布方:
        循环发送消息

"""
import rospy
from common.msg import motor


if __name__ == "__main__":
    #1.初始化 ROS 节点
    rospy.init_node("motor_pub")
    #2.创建发布者对象
    pub = rospy.Publisher("motor_msg",motor,queue_size=10)
    #3.组织消息
    m=motor
    m.num=20;
    m.PWM=90;

    #4.编写消息发布逻辑
    rate = rospy.Rate(0.8)
    while not rospy.is_shutdown():
        pub.publish(m)  #发布消息
        rate.sleep()  #休眠

