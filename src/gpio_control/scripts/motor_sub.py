import rospy
import RPi.GPIO as GPIO
import time
from common.msg import MultiMotorCommand, motor

# 设定控制电机的 GPIO 引脚
pwm_pin = [5, 6, 12, 13, 16, 19, 20, 21]

def doMultiMotor(msg):
    for m in msg.motors:
        rospy.loginfo("Received motor msg: num=%d, PWM=%.2f, time=%.2f", m.num, m.PWM, m.vibration_time)
        
        motor_pin = m.num  # 直接使用 BCM 编码
        pwm = GPIO.PWM(motor_pin, 320)  # 320Hz PWM频率
        pwm.start(m.PWM)  # 设置初始的PWM值
        
        # 振动持续时间
        time.sleep(m.vibration_time)  # 振动指定的时间
        
        pwm.ChangeDutyCycle(0)  # 停止震动
        pwm.stop()  # 释放PWM资源

if __name__ == "__main__":
    # 1. 初始化 ROS 节点
    rospy.init_node("motor_sub")
    
    # 2. 设置 GPIO
    GPIO.setmode(GPIO.BCM)  # 使用 BCM 编码方式
    GPIO.setup(pwm_pin, GPIO.OUT)  # 设置 GPIO 引脚为输出
    
    # 3. 创建订阅者对象
    sub = rospy.Subscriber("multi_motor_msg", MultiMotorCommand, doMultiMotor, queue_size=10)
    
    # 4. 循环等待消息
    rospy.spin()
    
    # 5. 清理GPIO设置
    GPIO.cleanup()
