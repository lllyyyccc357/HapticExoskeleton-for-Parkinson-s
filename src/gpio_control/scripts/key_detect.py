#! /usr/bin/env python
"""
    订阅方:
        订阅消息

"""
import rospy


import RPi.GPIO as GPIO  #导入gpio口驱动模块
import time              #导入时间模块

key_pin = {18:"red",23:"green",24:"blue"}

def key_callBack(channel):
    rospy.loginfo(key_pin[channel])
    # add something

if __name__ == "__main__":
    #1.初始化节点
    rospy.init_node("ket_control")
    #2.创建订阅者对象

    GPIO.setmode(GPIO.BCM)   #定义树莓派gpio引脚以BCM方式编号
    GPIO.setup(18,GPIO.IN)  
    GPIO.setup(23,GPIO.IN)  
    GPIO.setup(24,GPIO.IN)  
    GPIO.add_event_detect(18, GPIO.RISING, callback=key_callBack)
    GPIO.add_event_detect(23, GPIO.RISING, callback=key_callBack)
    GPIO.add_event_detect(24, GPIO.RISING, callback=key_callBack)
    while True:
        pass  
    GPIO.cleanup()  # 清理GPIO配置
