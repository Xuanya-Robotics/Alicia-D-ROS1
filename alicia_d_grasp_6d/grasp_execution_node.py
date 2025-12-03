#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
抓取执行节点：接收检测到的物体位姿，控制机械臂执行抓取动作。
"""
import os
import sys

# 确保 ROS path 在 sys.path 中（conda 环境需要）
ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
if ros_path not in sys.path:
    sys.path.insert(0, ros_path)

# 导入 ROS 相关的包（必须在导入 moveit_control 之前）
import rospy
from geometry_msgs.msg import PoseStamped

# 添加 MoveIt 控制模块路径
robot_path = '/home/ubuntu/alicia_ws/src/alicia_d_moveit/scripts'
if robot_path not in sys.path:
    sys.path.append(robot_path)

# 导入 MoveIt 控制器（需要在 ROS 环境中，必须通过 rosrun 运行）
try:
    from moveit_control import MoveItRobotController
except ImportError as e:
    rospy.logerr('=' * 80)
    rospy.logerr('无法导入 MoveItRobotController: %s', e)
    rospy.logerr('=' * 80)
    rospy.logerr('请确保：')
    rospy.logerr('  1. 使用 rosrun 运行节点（不是直接用 python 运行）')
    rospy.logerr('  2. ROS 环境已设置: source /opt/ros/noetic/setup.bash')
    rospy.logerr('  3. 工作空间已编译: cd ~/alicia_ws && catkin_make')
    rospy.logerr('  4. 工作空间已 source: source ~/alicia_ws/devel/setup.bash')
    rospy.logerr('=' * 80)
    MoveItRobotController = None
except Exception as e:
    rospy.logerr('导入 MoveItRobotController 时发生错误: %s', e)
    import traceback
    rospy.logerr(traceback.format_exc())
    MoveItRobotController = None

# 导入所有 ROS 包后，移除 ROS path 避免与 conda 环境中的 cv2 冲突
# 注意：moveit_control 已经导入，所以可以安全移除
if ros_path in sys.path:
    sys.path.remove(ros_path)


class GraspExecutionNode:
    """订阅检测到的物体位姿，执行抓取动作"""
    
    def __init__(self):
        # 订阅检测到的物体位姿话题
        self.pose_topic = rospy.get_param(
            '~pose_topic', '/graspnet_pointcloud_node/detected_object_pose'
        )
        
        # 抓取参数（方案一：直接移动到目标位姿）
        self.gripper_close_value = rospy.get_param('~gripper_close_value', 1.8)  # 夹爪闭合值
        self.gripper_close_iterations = int(rospy.get_param('~gripper_close_iterations', 10))  # 夹爪闭合迭代次数
        self.require_user_confirmation = rospy.get_param('~require_user_confirmation', True)  # 是否需要用户确认
        
        # 初始化 MoveIt 控制器
        if MoveItRobotController is None:
            rospy.logerr('=' * 80)
            rospy.logerr('MoveItRobotController 不可用，无法执行抓取')
            rospy.logerr('=' * 80)
            rospy.logerr('请检查：')
            rospy.logerr('  1. 是否在 ROS 环境中运行（使用 rosrun 或确保已 source ROS setup.bash）')
            rospy.logerr('  2. 工作空间是否已编译')
            rospy.logerr('  3. 文件是否存在: %s/moveit_control.py', robot_path)
            rospy.logerr('=' * 80)
            raise RuntimeError('MoveItRobotController 不可用，请检查 ROS 环境设置')
        
        try:
            # 使用与 alicia_grasp.py 相同的参数初始化
            self.moveit_controller = MoveItRobotController(
                manipulator_group="Alicia",
                gripper_group="Gripper",
                velocity=1.0
            )
            rospy.loginfo('MoveIt 控制器初始化成功')
            
            # 确保规划参数已正确设置（MoveItRobotController 内部已设置，但我们可以验证）
            try:
                manipulator = self.moveit_controller.manipulator
                rospy.loginfo('MoveIt 规划参数:')
                rospy.loginfo('  规划时间: %.1f 秒', manipulator.get_planning_time())
                rospy.loginfo('  位置容差: %.3f m', manipulator.get_goal_position_tolerance())
                rospy.loginfo('  姿态容差: %.3f rad', manipulator.get_goal_orientation_tolerance())
            except Exception as e:
                rospy.logwarn('无法获取部分规划参数: %s', e)
        except Exception as e:
            rospy.logerr('MoveIt 控制器初始化失败: %s', e)
            import traceback
            rospy.logerr(traceback.format_exc())
            raise
        
        # 订阅检测到的物体位姿
        self.sub = rospy.Subscriber(
            self.pose_topic, PoseStamped, self.pose_callback, queue_size=1
        )
        
        # 执行状态
        self.executing = False
        
        rospy.loginfo('=' * 80)
        rospy.loginfo('抓取执行节点已启动（方案一：直接移动到目标位姿）')
        rospy.loginfo('  订阅话题: %s', self.pose_topic)
        rospy.loginfo('  夹爪闭合值: %.2f', self.gripper_close_value)
        rospy.loginfo('  夹爪闭合迭代次数: %d', self.gripper_close_iterations)
        rospy.loginfo('  需要用户确认: %s', self.require_user_confirmation)
        rospy.loginfo('=' * 80)
    
    def wait_for_user_confirmation(self, message: str) -> bool:
        """
        等待用户确认：按 Enter 继续，或其他键跳过
        
        Args:
            message: 显示给用户的消息
            
        Returns:
            bool: True 如果用户按 Enter，False 如果跳过
        """
        print("\n" + "-" * 50)
        print(message + " (按 Enter 继续，或其他键跳过)")
        
        user_input = input()
        
        if user_input == "":
            print("继续执行抓取...")
            return True
        else:
            print("跳过此次抓取...")
            return False
    
    def execute_grasp(self, pose_stamped: PoseStamped) -> bool:
        """
        执行抓取动作
        
        Args:
            pose_stamped: 检测到的物体位姿（相对于base_link）
            
        Returns:
            bool: 抓取是否成功
        """
        if self.executing:
            rospy.logwarn('正在执行抓取，忽略新的位姿')
            return False
        
        self.executing = True
        
        try:
            # 用户确认（如果需要）
            if self.require_user_confirmation:
                pos = pose_stamped.pose.position
                rospy.loginfo('收到抓取目标位姿:')
                rospy.loginfo('  位置 (x, y, z): [%.6f, %.6f, %.6f] m', 
                             pos.x, pos.y, pos.z)
                if not self.wait_for_user_confirmation("准备执行抓取"):
                    self.executing = False
                    return False
            
            # 方案一：直接移动到目标位姿
            # 使用完整的 PoseStamped（包含 frame_id），确保坐标系正确
            rospy.loginfo('=' * 80)
            rospy.loginfo('开始执行抓取序列...')
            rospy.loginfo('=' * 80)
            rospy.loginfo('收到位姿消息:')
            rospy.loginfo('  frame_id: %s', pose_stamped.header.frame_id)
            rospy.loginfo('   位置: [%.6f, %.6f, %.6f]', 
                         pose_stamped.pose.position.x,
                         pose_stamped.pose.position.y,
                         pose_stamped.pose.position.z)
            rospy.loginfo('   姿态: [%.6f, %.6f, %.6f, %.6f]',
                         pose_stamped.pose.orientation.x,
                         pose_stamped.pose.orientation.y,
                         pose_stamped.pose.orientation.z,
                         pose_stamped.pose.orientation.w)
            
            # 检查并调整目标位姿的合理性
            min_z = 0.08  # 最小z高度（8cm）
            if pose_stamped.pose.position.z < min_z:
                rospy.logwarn('警告：目标z坐标太低 (%.4f m < %.4f m)，调整为 %.4f m', 
                             pose_stamped.pose.position.z, min_z, min_z)
                pose_stamped.pose.position.z = min_z
            
            # 打开夹爪
            rospy.loginfo('打开夹爪...')
            self.moveit_controller.close_gripper(0.0)
            rospy.sleep(0.5)
            
            # 检查当前位姿
            try:
                current_pose = self.moveit_controller.get_current_pose()
                rospy.loginfo('当前机械臂位姿:')
                rospy.loginfo('  位置: [%.3f, %.3f, %.3f]', 
                             current_pose.position.x, current_pose.position.y, current_pose.position.z)
            except Exception as e:
                rospy.logwarn('无法获取当前位姿: %s', e)
            
            # 调整规划参数以提高成功率（参考 alicia_grasp.py 的设置）
            try:
                manipulator = self.moveit_controller.manipulator
                # 增加规划时间和尝试次数
                manipulator.set_planning_time(15.0)  # 增加到15秒
                manipulator.set_num_planning_attempts(20)  # 增加到20次
                # 放宽容差（如果目标位姿难以精确到达）
                manipulator.set_goal_position_tolerance(0.02)  # 2cm
                manipulator.set_goal_orientation_tolerance(0.05)  # 约3度
                rospy.loginfo('已调整规划参数: 规划时间=15s, 尝试次数=20, 位置容差=2cm, 姿态容差=3度')
            except Exception as e:
                rospy.logwarn('无法调整规划参数: %s', e)
            
            rospy.loginfo('开始规划路径并移动到目标位姿...')
            rospy.loginfo('这可能需要一些时间，请耐心等待...')
            
            # 调用 MoveIt 移动到目标位姿（传递完整的 PoseStamped）
            grasp_success = self.moveit_controller.move_to_pose(pose_stamped)
            
            if not grasp_success:
                rospy.logerr('=' * 80)
                rospy.logerr('移动到抓取位置失败')
                rospy.logerr('可能的原因：')
                rospy.logerr('  1. 目标位姿超出工作空间')
                rospy.logerr('  2. 路径规划失败（碰撞或不可达）')
                rospy.logerr('  3. 规划时间不足')
                rospy.logerr('=' * 80)
                self.executing = False
                return False
            
            rospy.loginfo('抓取位置到达成功')
            rospy.sleep(0.5)
            
            # 闭合夹爪
            rospy.loginfo('闭合夹爪...')
            for i in range(self.gripper_close_iterations):
                self.moveit_controller.close_gripper(self.gripper_close_value)
                rospy.sleep(0.1)
            
            rospy.loginfo('夹爪闭合完成')
            rospy.sleep(0.5)
            
            # 移动回 home 位姿
            rospy.loginfo('移动回 home 位姿...')
            try:
                home_joint_values = self.moveit_controller.manipulator.get_named_target_values("home")
                rospy.loginfo('Home 关节值: %s', home_joint_values)
                if not self.moveit_controller.move_to_joint_state(home_joint_values):
                    rospy.logerr('返回 home 位姿失败')
                    self.executing = False
                    return False
            except Exception as e:
                rospy.logwarn('无法获取 home 位姿: %s', e)
            
            rospy.loginfo('=' * 80)
            rospy.loginfo('抓取执行完成！')
            rospy.loginfo('=' * 80)
            
            self.executing = False
            return True
            
        except Exception as e:
            rospy.logerr('抓取执行过程中发生错误: %s', e)
            import traceback
            rospy.logerr(traceback.format_exc())
            self.executing = False
            return False
    
    def pose_callback(self, msg: PoseStamped):
        """接收到检测到的物体位姿时的回调"""
        if self.executing:
            rospy.logwarn('正在执行抓取，忽略新的位姿')
            return
        
        rospy.loginfo('收到检测到的物体位姿')
        self.execute_grasp(msg)
    
    def spin(self):
        """运行节点"""
        rospy.spin()


def main():
    rospy.init_node('grasp_execution_node', anonymous=False)
    try:
        node = GraspExecutionNode()
        node.spin()
    except Exception as e:
        rospy.logerr('节点启动失败: %s', e)
        import traceback
        rospy.logerr(traceback.format_exc())


if __name__ == '__main__':
    main()

