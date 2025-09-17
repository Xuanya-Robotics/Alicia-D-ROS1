#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS节点：机器人姿态复制器

该节点用于读取之前记录的机器人关节状态数据(rosbag格式)，
并将其转换为命令直接发送给机器人控制器执行。
支持调整执行速度。
"""

import rospy
import rosbag
import os
import sys
from alicia_duo_driver.msg import ArmJointState
from std_msgs.msg import Float32


class PoseReplicator:
    """用于复现记录的机器人姿态轨迹的ROS节点类"""
    
    def __init__(self):
        """初始化姿态复制器节点"""
        rospy.init_node('pose_replicator', anonymous=True)

        # 从参数服务器获取参数
        self.bag_file_path = rospy.get_param('~bag_file', None)
        self.speed_factor = rospy.get_param('~speed_factor', 1.0)  # 速度因子，越大执行越快

        # 如果未指定bag文件路径，使用默认路径
        if not self.bag_file_path:
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.bag_file_path = os.path.join(script_dir, 'pose_data.bag')

        # 创建发布者，分别用于机械臂关节和夹爪
        # 使用直接控制话题，不需要MoveIt
        self.joint_pub = rospy.Publisher(
            '/arm_joint_command', 
            ArmJointState, 
            queue_size=10
        )
        self.gripper_pub = rospy.Publisher(
            '/gripper_control', 
            Float32, 
            queue_size=10
        )

        # 输出初始化信息
        rospy.loginfo("姿态复制器初始化完成")
        rospy.loginfo("Bag文件路径：%s", self.bag_file_path)
        rospy.loginfo("执行速度因子: %.2f", self.speed_factor)

    def load_trajectory_from_bag(self):
        """
        从bag文件中加载轨迹数据
        
        Returns:
            tuple: (joint_positions, timestamps) 联合位置数组和对应的时间戳数组
                  如果加载失败则返回None
        """
        # 检查文件是否存在
        if not os.path.exists(self.bag_file_path):
            rospy.logerr("Bag文件不存在: %s", self.bag_file_path)
            return None

        try:
            # 打开bag文件
            bag = rosbag.Bag(self.bag_file_path)
            topic_name = '/recorded_arm_joint_state'
            joint_positions = []
            timestamps = []

            # 读取所有消息
            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                joint_pos = [
                    msg.joint1, msg.joint2, msg.joint3,
                    msg.joint4, msg.joint5, msg.joint6,
                    msg.gripper
                ]
                joint_positions.append(joint_pos)
                timestamps.append(t.to_sec())

            bag.close()

            # 检查是否成功读取数据
            if not joint_positions:
                rospy.logwarn("未在主题 %s 中读取到任何轨迹点", topic_name)
                return None

            rospy.loginfo("成功读取 %d 个轨迹点", len(joint_positions))
            return joint_positions, timestamps

        except Exception as e:
            rospy.logerr("读取轨迹数据失败: %s", str(e))
            return None

    def send_trajectory_directly(self, joint_positions, timestamps):
        """
        直接发送关节命令
        
        Args:
            joint_positions: 关节位置数组列表
            timestamps: 时间戳列表
            
        Returns:
            bool: 是否成功发送轨迹
        """
        if not joint_positions or not timestamps:
            rospy.logerr("轨迹数据无效")
            return False

        try:
            # 总轨迹时间（考虑速度因子）
            total_time = (timestamps[-1] - timestamps[0]) / self.speed_factor
            rospy.loginfo("开始执行轨迹，总时长：%.2f秒", total_time)
            
            # 等待发布者连接
            rospy.sleep(1.0)
            
            start_time = rospy.Time.now().to_sec()
            last_index = 0
            
            # 循环直到轨迹执行完成
            rate = rospy.Rate(50)  # 50Hz控制频率
            while not rospy.is_shutdown():
                current_time = rospy.Time.now().to_sec()
                elapsed = current_time - start_time
                
                # 检查是否完成
                if elapsed > total_time:
                    # 发送最后一个点
                    self._send_joint_command(joint_positions[-1])
                    break
                
                # 计算当前应该执行的索引
                # 根据经过的时间和速度因子计算
                scaled_elapsed = elapsed * self.speed_factor
                target_time = timestamps[0] + scaled_elapsed
                
                # 查找最接近的时间点
                for i in range(last_index, len(timestamps)):
                    if timestamps[i] >= target_time:
                        # 发送这个点的命令
                        self._send_joint_command(joint_positions[i])
                        last_index = i
                        break
                    
                # 保持循环频率
                rate.sleep()
            
            rospy.loginfo("轨迹执行完成")
            return True
            
        except Exception as e:
            rospy.logerr("发送轨迹失败: %s", str(e))
            return False
    
    def _send_joint_command(self, joint_pos):
        """
        发送单个关节位置命令
        
        Args:
            joint_pos: 包含7个关节角度的列表
        """
        # 创建并发送机械臂命令
        arm_msg = ArmJointState()
        arm_msg.joint1 = joint_pos[0]
        arm_msg.joint2 = joint_pos[1]
        arm_msg.joint3 = joint_pos[2]
        arm_msg.joint4 = joint_pos[3]
        arm_msg.joint5 = joint_pos[4]
        arm_msg.joint6 = joint_pos[5]
        self.joint_pub.publish(arm_msg)
        
        # 创建并发送夹爪命令
        gripper_msg = Float32()
        gripper_msg.data = joint_pos[6]
        self.gripper_pub.publish(gripper_msg)

    def run(self):
        """
        运行姿态复制器主程序
        
        Returns:
            bool: 操作是否成功
        """
        # 加载轨迹数据
        data = self.load_trajectory_from_bag()
        if not data:
            rospy.logerr("无法加载轨迹数据，退出")
            return False

        joint_positions, timestamps = data

        # 提示用户开始执行
        rospy.loginfo("按 Enter 键开始执行轨迹...")
        try:
            input()  # Python 3
        except NameError:
            raw_input()  # Python 2 兼容

        # 发送轨迹
        return self.send_trajectory_directly(joint_positions, timestamps)


def main():
    """主函数"""
    try:
        replicator = PoseReplicator()
        success = replicator.run()
        
        if success:
            rospy.loginfo("轨迹执行完成")
        else:
            rospy.logerr("轨迹执行失败")
            
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS节点被中断")
    except KeyboardInterrupt:
        rospy.loginfo("操作被用户取消")
    except Exception as e:
        rospy.logerr("姿态复制器发生未预期异常: %s", str(e))


if __name__ == '__main__':
    main()

