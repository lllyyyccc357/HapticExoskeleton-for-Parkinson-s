#!/usr/bin/env python
import rospy
import RPi.GPIO as GPIO
import time

def get_motor_pin_map():
    """
    从参数服务器加载 motor_outA ~ motor_outH 的电机引脚映射
    返回 {'a':5, 'b':6, ...}
    """
    motor_map = {}
    for k in "ABCDEFGH":
        param_name = f"motor_out{k}"
        if rospy.has_param(param_name):
            motor_map[k.lower()] = rospy.get_param(param_name)
        else:
            rospy.logwarn(f"参数 {param_name} 不存在！")
    return motor_map

def main():
    # 初始化 ROS 节点
    rospy.init_node("multi_motor_interactive_test", anonymous=True)

    # 获取电机映射表 {'a':5, 'b':6, ...}
    motor_map = get_motor_pin_map()
    if not motor_map:
        rospy.logerr("未加载到任何电机参数，程序退出！")
        return

    # 初始化 GPIO
    GPIO.setmode(GPIO.BCM)
    for pin in motor_map.values():
        GPIO.setup(pin, GPIO.OUT)

    # 模式定义
    modes = {
        "1": {"PWM": 50, "time": 0.5},
        "2": {"PWM": 80, "time": 1.0},
    }

    # 主循环
    while not rospy.is_shutdown():
        keys = input("请输入电机编号组合 (a-h，例如 ab 或 acf，q退出): ").strip().lower()
        if keys == "q":
            break

        selected_pins = []
        valid_keys = []
        for k in keys:
            k_lower = k.lower()
            if k_lower in motor_map:
                selected_pins.append(motor_map[k_lower])
                valid_keys.append(k_lower)
            else:
                print(f"无效电机编号: {k}")

        if not selected_pins:
            print("没有有效的电机编号，请重新输入")
            continue

        mode = raw_input("选择模式 (1 or 2): ").strip()
        if mode not in modes:
            print("模式错误，请输入1或2！")
            continue

        pwm_value = modes[mode]["PWM"]
        duration = modes[mode]["time"]

        # 创建 PWM 对象
        pwms = []
        for pin in selected_pins:
            p = GPIO.PWM(pin, 320)
            p.start(0)
            pwms.append(p)

        print(f"开始震动电机 {[k.upper() for k in valid_keys]}，模式 {mode}，5 次循环")
        for i in range(5):
            print(f"第 {i+1} 次震动")
            for p in pwms:
                p.ChangeDutyCycle(pwm_value)
            time.sleep(duration)
            for p in pwms:
                p.ChangeDutyCycle(0)
            time.sleep(0.5)

        for p in pwms:
            p.stop()

        print("完成5次震动")

    GPIO.cleanup()
    print("退出程序，GPIO已清理")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        GPIO.cleanup()
