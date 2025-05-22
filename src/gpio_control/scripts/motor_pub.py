#! /usr/bin/env python
import rospy
import random
from common.msg import motor

if __name__ == "__main__":
    # 1.初始化 ROS 节点
    rospy.init_node("motor_pub")
    
    # 2.创建发布者对象
    pub = rospy.Publisher("motor_msg", motor, queue_size=10)
    
    # 3.编写消息发布逻辑
    rate = rospy.Rate(0.8)  # 0.8 Hz，即每1.25秒发送一次
    
    while not rospy.is_shutdown():
        m = motor()
        # 随机生成电机编号、PWM 和振动时间
        m.num = random.choice([5, 6, 12, 13, 16, 19, 20, 21])  # 随机选择一个电机编号
        m.PWM = random.uniform(20, 100)  # 随机生成 PWM 数值
        m.vibration_time = random.uniform(0.5, 3.0)  # 随机生成振动时间
        
        rospy.loginfo(f"Publishing motor msg: num={m.num}, PWM={m.PWM}, time={m.vibration_time}")
        
        # 发布消息
        pub.publish(m)
        
        # 休眠
        rate.sleep()
