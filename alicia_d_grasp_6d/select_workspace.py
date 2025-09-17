import os
import sys
import numpy as np
import argparse
from cv_bridge import CvBridge

from PIL import Image
import time
import scipy.io as scio
import torch
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
from dataset.graspnet_dataset import minkowski_collate_fn
from collision_detector import ModelFreeCollisionDetector
from data_utils import CameraInfo, create_point_cloud_from_depth_image, get_workspace_mask
from UR_Robot import UR_Robot
import moveit_commander
from geometry_msgs.msg import PoseStamped
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix
)

def select_workspace(image):
    """
    使用鼠标点击选择工作区域，四个点围成四边形。
    :param image: 输入图像
    :return: 工作区域掩膜, 四个顶点坐标
    """
    points = []
    mask = None  # 初始化掩膜

    def order_points_clockwise(pts):
        """
        按顺时针顺序排列点
        :param pts: 输入点 (x, y)
        :return: 排序后的点
        """
        rect = np.zeros((4, 2), dtype="float32")

        # 找到左上角和右下角点
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # 左上角
        rect[2] = pts[np.argmax(s)]  # 右下角

        # 找到右上角和左下角点
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # 右上角
        rect[3] = pts[np.argmax(diff)]  # 左下角

        return rect

    def mouse_callback(event, x, y, flags, param):
        nonlocal points, mask
        if event == cv2.EVENT_LBUTTONDOWN:
            # 左键点击添加顶点
            points.append((x, y))
            cv2.circle(temp_image, (x, y), 5, (0, 0, 255), -1)  # 在图像上画点
            cv2.imshow('Select Workspace', temp_image)
            if len(points) == 4:
                # 检查点的有效性
                if len(set(points)) != 4:
                    print("错误：选择的点不唯一，请重新选择！")
                    points.clear()
                    return

                # 确保点按顺时针顺序排列
                ordered_points = order_points_clockwise(np.array(points, dtype=np.float32))
                mask = np.zeros(image.shape[:2], dtype=np.uint8)
                cv2.fillPoly(mask, [np.array(ordered_points, dtype=np.int32)], 255)  # 填充四边形区域
                print("工作区域选择完成。")
                print("选择的点坐标:", ordered_points)
                cv2.destroyAllWindows()

    # 创建用于显示的临时图像
    temp_image = image.copy()
    cv2.imshow('Select Workspace', temp_image)
    cv2.setMouseCallback('Select Workspace', mouse_callback)
    print("请用左键点击依次选择四个顶点，确定工作区域。")

    # 等待用户完成选择
    while True:
        if mask is not None:  # 确保掩膜已生成
            break
        if cv2.waitKey(1) & 0xFF == 27:  # 按下 ESC 键退出
            print("选择取消。")
            cv2.destroyAllWindows()
            break

    # 如果用户未完成选择，返回空掩膜
    if mask is None:
        print("未生成有效的工作区域掩膜，返回默认掩膜。")
        mask = np.zeros(image.shape[:2], dtype=np.uint8)
        points = []
    print(points)
    return mask

#main
if __name__ == '__main__':
    rospy.init_node("select_workspace", anonymous=True)
    
    # ROS 桥接器，用于将 ROS 图像消息转换为 OpenCV 格式
    bridge = CvBridge()
    
    # 订阅 RealSense 相机的相关话题
    color_topic = "/camera/color/image_raw"  # 彩色图像话题
    depth_topic = "/camera/depth/image_rect_raw"  # 深度图像话题
    color_camera_info_topic = "/camera/color/camera_info"  # 彩色相机信息话题
    depth_camera_info_topic = "/camera/depth/camera_info"  # 深度相机信息话题

    # 将 ROS 消息转换为 OpenCV 格式
    color_msg = rospy.wait_for_message(color_topic, Image)
    depth_msg = rospy.wait_for_message(depth_topic, Image)
    
    # 归一化并预处理图像
    rgb = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")  # 转换彩色图像
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float32)  # 转换深度图像
    
    # 将颜色值归一化到 [0, 1]
    color = rgb / 255.0
    color = color[:, :, ::-1]  # 将 RGB 转换为 BGR 格式
    # 获取相机的内参信息
    depth_camera_info = rospy.wait_for_message(depth_camera_info_topic, ROSCameraInfo)
    
    # 解析深度相机的内参矩阵
    intrinsic = np.array(depth_camera_info.K).reshape(3, 3)
    factor_depth = 1.0 / 0.0010000000474974513  # 深度缩放因子
    
    # 创建相机信息对象
    camera = CameraInfo(
        depth_camera_info.width,
        depth_camera_info.height,
        intrinsic[0, 0],
        intrinsic[1, 1],
        intrinsic[0, 2],
        intrinsic[1, 2],
        factor_depth
    )
    
    workspace_mask = select_workspace(rgb)