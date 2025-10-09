#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
import math

def main():
    rospy.init_node('alicia_control_example')
    
    # 创建发布器
    joint_pub = rospy.Publisher('/joint_commands', JointState, queue_size=10)
    
    # 等待ROS系统初始化
    rospy.sleep(1.0)
    
    # 创建关节状态消息
    joint_msg = JointState()
    joint_msg.header.stamp = rospy.Time.now()
    joint_msg.name = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6', 'right_finger']
    
    # 测试序列
    test_positions = [
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 归零位置
        [0.5, -0.3, 0.8, 0.0, 0.2, 0.0, 0.02], # 测试位置1
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.04],  # 归零并打开夹爪
    ]
    
    for i, positions in enumerate(test_positions):
        rospy.loginfo(f"执行测试位置 {i+1}")
        joint_msg.position = positions
        joint_msg.header.stamp = rospy.Time.now()
        joint_pub.publish(joint_msg)
        rospy.sleep(4.0)  # 等待运动完成
    
    rospy.loginfo("示例程序执行完成")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass