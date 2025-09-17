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


    def __init__(self, manipulator_group="alicia", gripper_group="hand", velocity=1.0):
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
        self.gripper = moveit_commander.MoveGroupCommander(gripper_group)
        self.robot_name = self.manipulator.get_active_joints()
        # Set velocity scaling (0.0 to 1.0)
        self.manipulator.set_max_velocity_scaling_factor(velocity)
        # self.gripper.set_max_velocity_scaling_factor(velocity)
        # set the manximum acceleration scaling factor
        self.manipulator.set_max_acceleration_scaling_factor(0.5)
        # Match planner id with ompl_planning.yaml to avoid warnings
        self.manipulator.set_planner_id("RRTConnectkConfigDefault")
        self.manipulator.set_planning_time(10.0)                     # 增加规划时间
        self.manipulator.set_num_planning_attempts(10)
        self.manipulator.set_goal_position_tolerance(0.01)
        self.manipulator.set_goal_orientation_tolerance(0.01)
        self.manipulator.allow_replanning(True)



    def move_to_pose(self, pose):
        self.manipulator.set_pose_target(pose)
        success = self.manipulator.go(wait=True)
        self.manipulator.stop()
        self.manipulator.clear_pose_targets()
        return success
    
    def open_gripper(self):
        self.gripper.set_joint_value_target([0.0])
        success = self.gripper.go(wait=True)
        self.gripper.stop()
        self.gripper.clear_pose_targets()
        return success

    def close_gripper(self, distance=0.05):
        self.gripper.set_joint_value_target([distance])
        success = self.gripper.go(wait=True)
        self.gripper.stop()
        self.gripper.clear_pose_targets()
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
        # rospy.loginfo("Moving to joint state: %s", joint_goals)
        # # print the type of joint_goals
        # rospy.loginfo("Type of joint_goals: %s", type(joint_goals))
        # Use MoveGroupCommander joint target API
        self.manipulator.set_joint_value_target(joint_goals)
        success = self.manipulator.go(wait=True)
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
    import time
    controller = MoveItRobotController()

    # 定义初始位姿（当前）
    start_pose = controller.manipulator.get_current_pose().pose
    rospy.loginfo("Current Pose: %s", start_pose)
    # Move to home joint state
    home_joint_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    success = controller.move_to_joint_state(home_joint_state)

    
    current_pose = controller.get_current_pose()
    rospy.loginfo("Current Pose: %s", current_pose)
    current_joint_state = controller.get_current_joint_state()
    # rospy.loginfo("Current Joint State: %s", current_joint_state)


    # goal_pose = Pose()
    # goal_pose.position.x = 0.2509282645230489
    # goal_pose.position.y = 0.17561738522586529
    # goal_pose.position.z = 0.2597247757446909
    # goal_pose.orientation.x = -0.0026896244049182307
    # goal_pose.orientation.y = 0.9679273860982205
    # goal_pose.orientation.z = -0.030580947902249436
    # goal_pose.orientation.w = 0.24934744190993252

    # success = controller.move_to_pose(goal_pose)


    """
    x: 0.3562864818655051
    y: 0.12518864867209947
    z: 0.12435312631374076
    orientation: 
    x: 0.016529085222138674
    y: 0.9480846094333752
    z: -0.017864083885818078
    w: 0.3170855361005991
    """
    goal_pose2 = Pose()
    goal_pose2.position.x = 0.3562864818655051
    goal_pose2.position.y = 0.12518864867209947
    goal_pose2.position.z = 0.12435312631374076
    goal_pose2.orientation.x = 0.016529085222138674
    goal_pose2.orientation.y = 0.9480846094333752
    goal_pose2.orientation.z = -0.017864083885818078
    goal_pose2.orientation.w = 0.3170855361005991

    success = controller.move_to_pose(goal_pose2)
    goal_pose = goal_pose2


    # import time
    time.sleep(1)
    current_pose = controller.get_current_pose()
    rospy.loginfo("Current Pose: %s", current_pose)
    current_joint_state = controller.get_current_joint_state()
    # rospy.loginfo("Current Joint State: %s", current_joint_state)

    # Position error (Euclidean distance)
    pos_err_vec = np.array([
        current_pose.position.x - goal_pose.position.x,
        current_pose.position.y - goal_pose.position.y,
        current_pose.position.z - goal_pose.position.z,
    ])
    pos_err_norm = float(np.linalg.norm(pos_err_vec))

    # Orientation error as angular distance between quaternions (radians)
    q_current = np.array([
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w,
    ])
    q_goal = np.array([
        goal_pose.orientation.x,
        goal_pose.orientation.y,
        goal_pose.orientation.z,
        goal_pose.orientation.w,
    ])
    # Ensure unit quaternions to get a valid angle
    def _safe_normalize(q):
        n = np.linalg.norm(q)
        return q / n if n > 0 else q
    q_current = _safe_normalize(q_current)
    q_goal = _safe_normalize(q_goal)
    dot = float(np.clip(np.abs(np.dot(q_current, q_goal)), 0.0, 1.0))
    ori_err_angle = float(2.0 * np.arccos(dot))

    rospy.loginfo("Position error (m): %s (vec=%s)", pos_err_norm, pos_err_vec.tolist())
    rospy.loginfo("Orientation error (rad): %s", ori_err_angle)


