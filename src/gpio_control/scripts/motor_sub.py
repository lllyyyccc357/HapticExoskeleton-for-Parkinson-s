#! /usr/bin/env python
"""
    订阅方:
        订阅消息

"""
import rospy
from common.msg import motor

import RPi.GPIO as GPIO  #导入gpio口驱动模块
import time              #导入时间模块

pwm_pin = [5,6, 12, 13, 16, 19, 20, 21]

def doMotor(m):
    rospy.loginfo("motor msg: %d, %.2f",m.num,m.PWM)
    pwm=GPIO.PWM(m.num,320)
    pwm.start(m.PWM)
    time.sleep(0.5)
    pwm.ChangeDutyCycle(0)



if __name__ == "__main__":
    #1.初始化节点
    rospy.init_node("motor_sub")
    #2.创建订阅者对象

    GPIO.setmode(GPIO.BCM)   #定义树莓派gpio引脚以BCM方式编号
    GPIO.setup(pwm_pin,GPIO.OUT)  #使能gpio口为输出
    for i in range(0,5):
        pwm = GPIO.PWM(pwm_pin[i],320)   #定义pwm输出频率

    sub = rospy.Subscriber("motor_msg",motor,doMotor,queue_size=10)
    rospy.spin() #4.循环
