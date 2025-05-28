#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time

def get_motor_num(key):
    param_name = f"motor_out{key.upper()}"
    if rospy.has_param(param_name):
        return rospy.get_param(param_name)
    else:
        print(f"参数 {param_name} 不存在！")
        return None

def main():
    # 初始化 ROS 节点
    rospy.init_node("motor_interactive_test", anonymous=True)

    # 设置 GPIO
    GPIO.setmode(GPIO.BCM)
    
    # 获取所有电机引脚
    motor_pins = []
    for key in "ABCDEFGH":
        pin = get_motor_num(key)
        if pin is not None:
            motor_pins.append(pin)
    GPIO.setup(motor_pins, GPIO.OUT)
    
    # 模式定义
    modes = {
        "1": {"PWM": 50, "time": 0.5},
        "2": {"PWM": 80, "time": 1.0},
    }

    # 主循环
    while not rospy.is_shutdown():
        key = raw_input("请输入电机编号 (a-h, q退出): ").strip().lower()
        if key == "q":
            break
        motor_pin = get_motor_num(key)
        if motor_pin is None:
            continue
        
        mode = raw_input("选择模式 (1 or 2): ").strip()
        if mode not in modes:
            print("模式错误，请输入1或2！")
            continue
        
        pwm_value = modes[mode]["PWM"]
        duration = modes[mode]["time"]

        # 创建PWM
        pwm = GPIO.PWM(motor_pin, 320)
        pwm.start(0)
        
        print(f"开始振动电机 {key.upper()} (GPIO {motor_pin})，模式 {mode}，5次循环")
        for i in range(5):
            print(f"第 {i+1} 次震动")
            pwm.ChangeDutyCycle(pwm_value)
            time.sleep(duration)
            pwm.ChangeDutyCycle(0)
            time.sleep(0.5)  # 每次震动间隔

        pwm.stop()
        print("完成5次震动")

    GPIO.cleanup()
    print("退出程序，GPIO已清理")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
