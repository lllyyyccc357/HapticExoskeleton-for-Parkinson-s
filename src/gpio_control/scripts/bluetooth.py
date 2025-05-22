#!/usr/bin/env python3

import rospy
import bluetooth
from std_msgs.msg import String  # 根据你订阅的话题类型修改这里的消息类型
import time

# Bluetooth server setup
class BluetoothCommunication:
    def __init__(self, bt_address, bt_port=1, retry_interval=5):
        self.bt_address = bt_address
        self.bt_port = bt_port
        self.retry_interval = retry_interval  # 连接失败时的重试间隔（秒）
        self.sock = None

    def connect(self):
        """与蓝牙设备建立连接，失败时持续尝试"""
        self.sock = bluetooth.BluetoothSocket(bluetooth.RFCOMM)
        rospy.loginfo("Attempting to connect to Bluetooth device...")
        while not rospy.is_shutdown():
            try:
                self.sock.connect((self.bt_address, self.bt_port))
                rospy.loginfo("Bluetooth connected!")
                return  # 成功连接后退出循环
            except bluetooth.BluetoothError as e:
                rospy.logwarn("Bluetooth connection failed: %s", e)
                rospy.loginfo("Retrying in %d seconds...", self.retry_interval)
                time.sleep(self.retry_interval)

    def send_data(self, data):
        """发送数据到蓝牙设备"""
        if self.sock:
            try:
                self.sock.send(data)
                rospy.loginfo("Sent data: %s", data)
            except bluetooth.BluetoothError as e:
                rospy.logerr("Error sending data: %s", e)

    def close(self):
        """关闭蓝牙连接"""
        if self.sock:
            self.sock.close()
            rospy.loginfo("Bluetooth connection closed.")


# ROS 话题回调函数
def callback(data, bt_comm):
    """当接收到ROS话题数据时，将数据发送到蓝牙设备"""
    message = data.data  # 获取消息数据，这里假设消息类型是std_msgs/String
    # rospy.loginfo("Received ROS message: %s", message)
    bt_comm.send_data(message)  # 通过蓝牙发送消息


def listener():
    rospy.init_node('bluetooth_publisher', anonymous=True)

    # 配置蓝牙设备地址（你需要替换为你实际设备的蓝牙地址）
    bt_address = "XX:XX:XX:XX:XX:XX"  # 替换成你设备的蓝牙地址
    bt_comm = BluetoothCommunication(bt_address)
    
    # 尝试连接蓝牙设备
    bt_comm.connect()

    # 订阅一个 ROS 话题（根据实际需要修改话题名称和消息类型）
    rospy.Subscriber('/your_topic', String, callback, bt_comm)
    
    rospy.loginfo("Ready to send messages over Bluetooth.")

    # 保持节点运行
    rospy.spin()

    # 关闭蓝牙连接
    bt_comm.close()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
