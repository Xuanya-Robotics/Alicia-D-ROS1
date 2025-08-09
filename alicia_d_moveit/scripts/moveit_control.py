#!/usr/bin/env python
import sys
import rospy
import moveit_commander
from geometry_msgs.msg import Pose
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import tf2_ros
import geometry_msgs.msg
import numpy as np
from sensor_msgs.msg import JointState

class MoveItRobotController:


    def __init__(self, manipulator_group="alicia", gripper_group="hand", velocity=0.6):
        # Initialize MoveIt
        moveit_commander.roscpp_initialize(sys.argv)
        if not rospy.get_node_uri():
            rospy.init_node('moveit_robot_controller', anonymous=True)  

        # Robot and planning interface
        self.manipulator_group_name = manipulator_group
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        # Move groups
        self.manipulator = moveit_commander.MoveGroupCommander(manipulator_group)
        # self.gripper = moveit_commander.MoveGroupCommander(gripper_group)
        self.robot_name = self.manipulator.get_active_joints()
        # Set velocity scaling (0.0 to 1.0)
        self.manipulator.set_max_velocity_scaling_factor(velocity)
        # self.gripper.set_max_velocity_scaling_factor(velocity)
        # set the manximum acceleration scaling factor
        self.manipulator.set_max_acceleration_scaling_factor(0.5)
        self.manipulator.set_planner_id("RRTConnectkConfigDefault")  # 更快更稳定
        self.manipulator.set_planning_time(10.0)                     # 增加规划时间
        self.manipulator.set_num_planning_attempts(10)
        self.manipulator.set_goal_position_tolerance(0.01)
        self.manipulator.set_goal_orientation_tolerance(0.01)
        self.manipulator.allow_replanning(True)



    def move_to_pose(self, pose):
        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        return success
    

    def get_current_pose(self):
        """
        获取当前机械臂的位姿
        
        Returns:
            geometry_msgs.msg.Pose: 当前机械臂的位姿
        """
        return self.manipulator.get_current_pose().pose
    


    def move_to_joint_state(self, joint_goals):
        # Transfer joint_goals to type of JointState
        joint_state = JointState()
        joint_state.name = self.robot_name
        joint_state.position = joint_goals
        rospy.loginfo("Moving to joint state: %s", joint_goals)
        # print the type of joint_goals
        rospy.loginfo("Type of joint_goals: %s", type(joint_goals))
        # success = self.manipulator.go(wait=True, joints=joint_state)
        success = self.manipulator.go(joints=joint_state)
        if not success:
            rospy.logwarn("Failed to move to joint state: %s", joint_goals)
            return False
        self.manipulator.stop()
        return success
    

    def get_current_joint_state(self):
        """
        获取当前机械臂的关节状态
        
        Returns:
            list: 当前机械臂的关节角度列表
        """
        return self.manipulator.get_current_joint_values()
    

if __name__ == '__main__':
    controller = MoveItRobotController()

    # 定义初始位姿（当前）
    start_pose = controller.manipulator.get_current_pose().pose
    rospy.loginfo("Current Pose: %s", start_pose)

    start_state = controller.manipulator.get_current_joint_values()
    rospy.loginfo("Current Joint State: %s", start_state)

    # Move to home joint state
    home_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    success = controller.move_to_joint_state(home_joint_state)
    if success:
        rospy.loginfo("Moved to home joint state successfully.")
    else:
        rospy.logwarn("Failed to move to home joint state.")

    test_joint_state = [0.8768841033096781, 0.03452299619329406, 0.9397926741507933, 0.4334553966491409, -1.4200459100841776, -0.8661436156050982]
    success = controller.move_to_joint_state(test_joint_state)
    if success:
        rospy.loginfo("Moved to test joint state successfully.")
    else:
        rospy.logwarn("Failed to move to test joint state.")