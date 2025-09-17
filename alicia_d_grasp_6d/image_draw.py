import numpy as np
import torch
import argparse
import sys
import os
from math import degrees  # 引入degrees函数
from geometry_msgs.msg import TransformStamped
from PIL import Image
import open3d as o3d
from graspnetAPI.graspnet_eval import GraspGroup
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo as ROSCameraInfo
import cv2
from scipy.spatial.transform import Rotation as R
from PIL import Image as PilImage
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
sys.path.append(os.path.join(ROOT_DIR, 'pointnet2'))
from models.graspnet import GraspNet, pred_decode
from data_utils import CameraInfo, create_point_cloud_from_depth_image
import moveit_commander
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from moveit_msgs.msg import  PlanningScene, ObjectColor,CollisionObject, AttachedCollisionObject,Constraints,OrientationConstraint
from geometry_msgs.msg import PoseStamped
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    quaternion_from_euler,
    euler_from_quaternion
)
import real.robotiq_gripper as robotiq_gripper
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion


ros_path = '/opt/ros/noetic/lib/python3/dist-packages'  # Adjust this path if necessary


sys.path.remove(ros_path)  # Remove ROS path after importing cv_bridge


from cv_bridge import CvBridge



def draw_2d_gripper(image, center_x, center_y, size=100, color=(0, 0, 255), angle=0):
    """
    Draw a 2D representation of a parallel gripper.
    
    Args:
        image: OpenCV image to draw on
        center_x, center_y: Position of the gripper
        size: Size of the gripper in pixels
        color: BGR color tuple for the gripper (default: blue)
        angle: Rotation angle in degrees
    """
    # Calculate gripper dimensions
    body_width = int(size * 0.6)
    body_height = int(size * 0.1)
    finger_width = int(size * 0.15)
    finger_length = int(size * 0.4)
    finger_gap = int(size * 0.3)
    
    # Create rotation matrix
    M = cv2.getRotationMatrix2D((center_x, center_y), angle, 1.0)
    
    # Define gripper body points
    body_points = np.array([
        [center_x - body_width//2, center_y - body_height//2],  # Top left
        [center_x + body_width//2, center_y - body_height//2],  # Top right
        [center_x + body_width//2, center_y + body_height//2],  # Bottom right
        [center_x - body_width//2, center_y + body_height//2]   # Bottom left
    ], dtype=np.float32).reshape(-1, 1, 2)
    
    # Rotate body points
    body_points = cv2.transform(body_points, M)
    
    # Define left finger points
    left_finger = np.array([
        [center_x - finger_gap//2 - finger_width//2, center_y + body_height//2],
        [center_x - finger_gap//2 + finger_width//2, center_y + body_height//2],
        [center_x - finger_gap//2 + finger_width//2, center_y + body_height//2 + finger_length],
        [center_x - finger_gap//2 - finger_width//2, center_y + body_height//2 + finger_length]
    ], dtype=np.float32).reshape(-1, 1, 2)
    
    # Define right finger points
    right_finger = np.array([
        [center_x + finger_gap//2 - finger_width//2, center_y + body_height//2],
        [center_x + finger_gap//2 + finger_width//2, center_y + body_height//2],
        [center_x + finger_gap//2 + finger_width//2, center_y + body_height//2 + finger_length],
        [center_x + finger_gap//2 - finger_width//2, center_y + body_height//2 + finger_length]
    ], dtype=np.float32).reshape(-1, 1, 2)
    
    # Rotate finger points
    left_finger = cv2.transform(left_finger, M)
    right_finger = cv2.transform(right_finger, M)
    
    # Draw filled shapes
    cv2.fillPoly(image, [np.int32(body_points)], color)
    cv2.fillPoly(image, [np.int32(left_finger)], color)
    cv2.fillPoly(image, [np.int32(right_finger)], color)



def gripper_draw():    
    # Load the RGB image
    rgb = cv2.imread("cropped_img.png")
    # save the color image
    color_image = PilImage.fromarray((rgb * 255).astype(np.uint8))
    color_image.save("color_image.png")

    rgb_draw = rgb.copy()
    
    # Example 1: Draw a horizontal line across the middle
    height, width = rgb_draw.shape[:2]
    # Draw a 2D representation of the gripper (blue like in the image)
    draw_2d_gripper(rgb_draw, width//2, height//2, size=90, color=(255, 0, 0), angle=-30)
    
    # Save the image with lines
    cv2.imwrite("image_with_lines.png", rgb_draw)
    

if __name__ == '__main__':
    gripper_draw()
    



