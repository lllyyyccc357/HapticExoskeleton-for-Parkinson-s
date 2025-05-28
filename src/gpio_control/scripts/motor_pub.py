#! /usr/bin/env python
import rospy
import random
from common.msg import MultiMotorCommand, motor

if __name__ == "__main__":
    # 1.初始化 ROS 节点
    rospy.init_node("motor_pub")
    
    # 2.创建发布者对象
    pub = rospy.Publisher("multi_motor_msg", MultiMotor, queue_size=10)
    
    # 3.编写消息发布逻辑
    rate = rospy.Rate(0.8)  # 0.8 Hz，即每1.25秒发送一次
    
    while not rospy.is_shutdown():
        msg = MultiMotor()
        motor_count = random.randint(1, 4)  # 每次发送1~4个电机控制指令
        
        for _ in range(motor_count):
            m = motor()
            m.num = random.choice([5, 6, 12, 13, 16, 19, 20, 21])
            m.PWM = random.uniform(20, 100)  # PWM值
            m.vibration_time = random.uniform(0.5, 3.0)  # 震动时间
            msg.motors.append(m)
        
        # 打印日志
        rospy.loginfo("Publishing MultiMotor msg:")
        for m in msg.motors:
            rospy.loginfo(f"  num={m.num}, PWM={m.PWM:.2f}, time={m.vibration_time:.2f}")
        
        # 发布消息raw_
        pub.publish(msg)
        
        # 休眠
        rate.sleep()
