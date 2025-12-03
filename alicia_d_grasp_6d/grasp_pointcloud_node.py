#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS 节点：订阅点云并调用 GraspNet (baseline) 进行 6D 抓取预测。
"""
import os
import sys
import struct
import threading
from typing import List, Tuple

import numpy as np
import torch
import yaml
import open3d as o3d
import cv2

# 确保 ROS path 在 sys.path 中（conda 环境需要）
ros_path = '/opt/ros/noetic/lib/python3/dist-packages'
if ros_path not in sys.path:
    sys.path.insert(0, ros_path)

# 导入所有 ROS 相关的包（必须在移除 ROS path 之前）
import rospy
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2, CameraInfo, Image
from geometry_msgs.msg import TransformStamped, PoseStamped
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_matrix, euler_from_quaternion
import tf2_ros

# 导入所有 ROS 包后，移除 ROS path 避免与 conda 环境中的 cv2 冲突
# 参考 alicia_grasp.py 的做法
if ros_path in sys.path:
    sys.path.remove(ros_path)


ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
GRASPNET_BASE_DIR = os.path.join(ROOT_DIR, 'graspnet-baseline')
# 权重文件默认路径
# 注意：checkpoint-rs.pth 是预训练模型（通用物体）
# checkpoint.tar 是微调后的模型（可能只针对红色方块，泛化能力下降）
DEFAULT_CHECKPOINT_FILE = os.path.join(
    GRASPNET_BASE_DIR, 'checkpoints', 'checkpoint.tar'
)
# 外参标定文件默认路径（easy_handeye 标定结果）
DEFAULT_EXTRINSIC_CALIB_FILE = os.path.expanduser(
    '~/.ros/easy_handeye/orbbec_eye_on_base_eye_on_base.yaml'
)

# 为 baseline 代码添加 Python path
sys.path.append(GRASPNET_BASE_DIR)
sys.path.append(os.path.join(GRASPNET_BASE_DIR, 'models'))
sys.path.append(os.path.join(GRASPNET_BASE_DIR, 'dataset'))
sys.path.append(os.path.join(GRASPNET_BASE_DIR, 'utils'))
sys.path.append(os.path.join(GRASPNET_BASE_DIR, 'pointnet2'))
sys.path.append(os.path.join(GRASPNET_BASE_DIR, 'graspnetAPI'))

from graspnet import GraspNet, pred_decode  # noqa: E402
from graspnetAPI import GraspGroup  # noqa: E402


class GraspNetInferenceNode:
    """订阅 PointCloud2 并输出 GraspNet 抓取候选。"""

    def __init__(self):
        self.point_topic = rospy.get_param(
            '~pointcloud_topic', '/camera/depth_registered/points'
        )
        self.num_point = int(rospy.get_param('~num_point', 10000))
        self.top_k = int(rospy.get_param('~top_k', 5))
        # 权重文件路径（使用文件开头定义的默认路径）
        self.checkpoint_path = rospy.get_param(
            '~checkpoint_path', DEFAULT_CHECKPOINT_FILE
        )
        
        # ROI裁剪参数（3D边界框，相机坐标系，单位：米）
        # None表示不裁剪该维度
        self.roi_enabled = rospy.get_param('~roi_enabled', True)
        self.roi_x_min = rospy.get_param('~roi_x_min', None)
        self.roi_x_max = rospy.get_param('~roi_x_max', None)
        self.roi_y_min = rospy.get_param('~roi_y_min', None)
        self.roi_y_max = rospy.get_param('~roi_y_max', None)
        self.roi_z_min = rospy.get_param('~roi_z_min', 0.1)  # 默认最小深度0.1m
        self.roi_z_max = rospy.get_param('~roi_z_max', 2.0)  # 默认最大深度2.0m
        
        # 颜色过滤参数（用于只识别特定颜色的物体，如红色方块）
        self.color_filter_enabled = rospy.get_param('~color_filter_enabled', True)
        self.target_color = rospy.get_param('~target_color', 'red')  # red, green, blue
        # HSV颜色范围（放宽阈值，保留更多相关颜色的点）
        self.color_ranges = {
            'red': {
                'lower1': np.array([0, 30, 30]),    # HSV下限1（红色在0度附近）- 放宽饱和度和亮度
                'upper1': np.array([15, 255, 255]),  # HSV上限1 - 扩大色相范围
                'lower2': np.array([165, 30, 30]),   # HSV下限2（红色在180度附近）- 放宽饱和度和亮度
                'upper2': np.array([180, 255, 255]) # HSV上限2
            },
            'green': {
                'lower': np.array([35, 30, 30]),     # 放宽绿色范围
                'upper': np.array([85, 255, 255])
            },
            'blue': {
                'lower': np.array([95, 30, 30]),     # 放宽蓝色范围
                'upper': np.array([135, 255, 255])
            }
        }
        
        if self.color_filter_enabled:
            rospy.loginfo('颜色过滤已启用，目标颜色: %s', self.target_color)
        
        if self.roi_enabled:
            rospy.loginfo('ROI裁剪已启用:')
            if self.roi_x_min is not None or self.roi_x_max is not None:
                rospy.loginfo('  X范围: [%s, %s]', 
                             self.roi_x_min if self.roi_x_min is not None else '-∞',
                             self.roi_x_max if self.roi_x_max is not None else '+∞')
            if self.roi_y_min is not None or self.roi_y_max is not None:
                rospy.loginfo('  Y范围: [%s, %s]',
                             self.roi_y_min if self.roi_y_min is not None else '-∞',
                             self.roi_y_max if self.roi_y_max is not None else '+∞')
            if self.roi_z_min is not None or self.roi_z_max is not None:
                rospy.loginfo('  Z范围: [%s, %s]',
                             self.roi_z_min if self.roi_z_min is not None else '-∞',
                             self.roi_z_max if self.roi_z_max is not None else '+∞')
        # 可视化参数（已禁用预览）
        self.visualize = rospy.get_param('~visualize', False)  # 默认禁用Open3D可视化
        self.visualize_top_k = int(rospy.get_param('~visualize_top_k', 5))
        self.visualize_once = rospy.get_param('~visualize_once', True)
        self.visualization_shown = False
        
        # RGB图像可视化参数（已禁用预览）
        self.rgb_image_topic = rospy.get_param('~rgb_image_topic', '/camera/color/image_raw')
        self.visualize_on_rgb = rospy.get_param('~visualize_on_rgb', False)  # 默认禁用RGB可视化
        self.publish_visualization = rospy.get_param('~publish_visualization', False)  # 默认不发布可视化图像
        self.vis_image_topic = rospy.get_param('~vis_image_topic', '/graspnet/visualization')
        
        # RGB图像相关
        self.rgb_image = None
        self.rgb_image_lock = threading.Lock()
        self.cv_bridge = CvBridge()
        
        # TF发布器（用于发布检测到的物体位置和相机外参）
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
        
        # 订阅RGB图像（用于可视化）
        if self.visualize_on_rgb:
            self.sub_rgb = rospy.Subscriber(
                self.rgb_image_topic, Image, self.rgb_image_callback, queue_size=1
            )
            if self.publish_visualization:
                self.pub_vis_image = rospy.Publisher(
                    self.vis_image_topic, Image, queue_size=1
                )
            rospy.loginfo('已订阅RGB图像话题: %s', self.rgb_image_topic)
        
        # 加载相机内参（从标准位置 ~/.ros/camera_info/ 自动加载，相机驱动会发布到话题）
        self.camera_info_topic = rospy.get_param(
            '~camera_info_topic', '/camera/color/camera_info'
        )
        self.camera_intrinsic = None
        self._load_camera_intrinsic()
        
        # 加载相机外参（相机到基座的变换）
        self.cam_frame_id = rospy.get_param('~camera_frame_id', 'camera_color_optical_frame')
        self.base_frame_id = rospy.get_param('~base_frame_id', 'base_link')
        self.use_tf_for_extrinsic = rospy.get_param('~use_tf_for_extrinsic', False)
        
        # 机械臂坐标系参数
        self.arm_frame_id = rospy.get_param('~arm_frame_id', 'link6')  # 机械臂末端坐标系
        self.use_tf_for_arm_transform = rospy.get_param('~use_tf_for_arm_transform', True)
        
        # TF相关（用于获取tool0位姿）
        self.tf_buffer = None
        self.tf_listener = None
        # 始终初始化TF监听器，用于获取tool0位姿
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.loginfo('已初始化TF监听器，用于获取tool0位姿')
        if self.use_tf_for_arm_transform:
            rospy.loginfo('已初始化TF监听器，用于查找机械臂坐标系: %s', self.arm_frame_id)
        # 外参标定文件路径（使用文件开头定义的默认路径）
        self.extrinsic_calib_file = rospy.get_param(
            '~extrinsic_calibration_file', DEFAULT_EXTRINSIC_CALIB_FILE
        )
        self._load_camera_extrinsic()
        
        # 发布 camera_color_optical_frame -> base_link 的静态TF（从外参文件读取）
        self._publish_camera_base_tf()

        if not os.path.isfile(self.checkpoint_path):
            raise FileNotFoundError(
                f'未找到 checkpoint: {self.checkpoint_path}'
            )

        self.device = torch.device(
            'cuda:0' if torch.cuda.is_available() else 'cpu'
        )
        self.net = self._load_model(self.checkpoint_path)
        self.net.eval()

        self.processing = False
        
        # 发布检测到的物体位置话题
        self.pose_pub = rospy.Publisher(
            '~detected_object_pose', PoseStamped, queue_size=1
        )
        
        # 缓冲区参数（用于稳定检测结果）
        self.buffer_size = int(rospy.get_param('~buffer_size', 3))  # 缓冲区大小（3个结果）
        self.pose_buffer = []  # 存储转换后的位姿（base_link坐标系）
        self.buffer_lock = threading.Lock()  # 缓冲区锁
        self.consecutive_outliers = 0  # 连续异常值计数（用于检测是否需要清空缓冲区）
        self.max_consecutive_outliers = int(rospy.get_param('~max_consecutive_outliers', 10))  # 最大连续异常值次数，超过后清空缓冲区
        
        # 滤波参数
        self.max_position_change = float(rospy.get_param('~max_position_change', 0.12))  # 最大位置变化（米），超过此值认为是异常值
        self.max_orientation_change = float(rospy.get_param('~max_orientation_change', 0.8))  # 最大姿态变化（弧度），超过此值认为是异常值
        
        self.sub = rospy.Subscriber(
            self.point_topic, PointCloud2, self.pointcloud_callback, queue_size=1
        )
        rospy.loginfo(
            'GraspNet 节点已启动，监听话题 %s，使用 checkpoint %s',
            self.point_topic,
            self.checkpoint_path,
        )
        rospy.loginfo('发布检测到的物体位置到话题: %s', self.pose_pub.resolved_name)

    def _load_camera_intrinsic(self):
        """加载相机内参：从话题订阅或从参数服务器读取"""
        # 优先从话题获取（动态）
        try:
            rospy.loginfo('等待相机内参话题: %s', self.camera_info_topic)
            camera_info_msg = rospy.wait_for_message(
                self.camera_info_topic, CameraInfo, timeout=5.0
            )
            K = np.array(camera_info_msg.K).reshape(3, 3)
            self.camera_intrinsic = {
                'fx': K[0, 0],
                'fy': K[1, 1],
                'cx': K[0, 2],
                'cy': K[1, 2],
                'width': camera_info_msg.width,
                'height': camera_info_msg.height
            }
            rospy.loginfo('从话题加载相机内参: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f',
                         self.camera_intrinsic['fx'], self.camera_intrinsic['fy'],
                         self.camera_intrinsic['cx'], self.camera_intrinsic['cy'])
        except rospy.ROSException:
            # 如果话题不存在，尝试从参数服务器读取
            rospy.logwarn('无法从话题获取内参，尝试从参数服务器读取...')
            fx = rospy.get_param('~camera_fx', None)
            fy = rospy.get_param('~camera_fy', None)
            cx = rospy.get_param('~camera_cx', None)
            cy = rospy.get_param('~camera_cy', None)
            if all(v is not None for v in [fx, fy, cx, cy]):
                self.camera_intrinsic = {
                    'fx': float(fx), 'fy': float(fy),
                    'cx': float(cx), 'cy': float(cy),
                    'width': int(rospy.get_param('~camera_width', 640)),
                    'height': int(rospy.get_param('~camera_height', 480))
                }
                rospy.loginfo('从参数服务器加载相机内参')
            else:
                rospy.logwarn('未找到相机内参，某些功能可能不可用')

    def _load_camera_extrinsic(self):
        """加载相机外参（相机到基座的变换）：优先级：文件 > TF树 > 参数服务器"""
        # 优先级1：从YAML文件加载
        if self.extrinsic_calib_file and os.path.isfile(self.extrinsic_calib_file):
            if self._load_extrinsic_from_file(self.extrinsic_calib_file):
                rospy.loginfo('从文件加载外参成功: %s', self.extrinsic_calib_file)
            else:
                rospy.logwarn('从文件加载外参失败，尝试其他方式...')
                self._load_extrinsic_from_tf_or_params()
        # 优先级2：从TF树或参数服务器读取
        else:
            self._load_extrinsic_from_tf_or_params()
        
        # 构建变换矩阵
        # 注意：cam_to_base_trans/quat 是从 camera_color_optical_frame 到 base_link 的变换
        # 所以构建的矩阵是 T_cam_base（camera -> base）
        self.T_cam_base = quaternion_matrix(self.cam_to_base_quat)
        self.T_cam_base[0, 3] = self.cam_to_base_trans[0]
        self.T_cam_base[1, 3] = self.cam_to_base_trans[1]
        self.T_cam_base[2, 3] = self.cam_to_base_trans[2]
        
        # 计算逆变换：base_link -> camera_color_optical_frame（用于TF发布）
        self.T_base_cam = np.linalg.inv(self.T_cam_base)

    def _load_extrinsic_from_file(self, file_path: str) -> bool:
        """从YAML文件加载外参标定"""
        try:
            with open(file_path, 'r') as f:
                calib_data = yaml.safe_load(f)
            
            # 支持多种YAML格式
            if 'transformation' in calib_data:
                # easy_handeye格式
                # 注意：transformation是从 tracking_base_frame (camera_color_optical_frame) 
                # 到 robot_base_frame (base_link) 的变换
                trans = calib_data['transformation']
                self.cam_to_base_trans = [trans['x'], trans['y'], trans['z']]
                self.cam_to_base_quat = [trans['qx'], trans['qy'], trans['qz'], trans['qw']]
                rospy.loginfo('从easy_handeye格式加载外参:')
                rospy.loginfo('  transformation方向: camera_color_optical_frame -> base_link')
                rospy.loginfo('  位置: [%.6f, %.6f, %.6f]', 
                             self.cam_to_base_trans[0], self.cam_to_base_trans[1], self.cam_to_base_trans[2])
                rospy.loginfo('  四元数: [%.6f, %.6f, %.6f, %.6f]',
                             self.cam_to_base_quat[0], self.cam_to_base_quat[1], 
                             self.cam_to_base_quat[2], self.cam_to_base_quat[3])
            elif 'camera_extrinsic' in calib_data:
                # 自定义格式
                ext = calib_data['camera_extrinsic']
                self.cam_to_base_trans = ext.get('translation', [0, 0, 0])
                self.cam_to_base_quat = ext.get('quaternion', [0, 0, 0, 1])
            elif 'translation' in calib_data and 'quaternion' in calib_data:
                # 直接格式
                self.cam_to_base_trans = calib_data['translation']
                self.cam_to_base_quat = calib_data['quaternion']
            else:
                rospy.logerr('YAML文件格式不支持: %s', file_path)
                return False
            
            rospy.loginfo('从文件加载外参: trans=%s, quat=%s',
                         self.cam_to_base_trans, self.cam_to_base_quat)
            return True
        except Exception as e:
            rospy.logerr('读取外参标定文件失败: %s, 错误: %s', file_path, e)
            return False

    def _load_extrinsic_from_tf_or_params(self):
        """从TF树或参数服务器加载外参"""
        if self.use_tf_for_extrinsic:
            # 从TF树获取
            rospy.loginfo('从TF树获取相机外参: %s -> %s', self.cam_frame_id, self.base_frame_id)
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer)
            try:
                rospy.sleep(1.0)  # 等待TF树建立
                transform = tf_buffer.lookup_transform(
                    self.base_frame_id, self.cam_frame_id, rospy.Time(0), timeout=rospy.Duration(5.0)
                )
                trans = transform.transform.translation
                rot = transform.transform.rotation
                self.cam_to_base_trans = [trans.x, trans.y, trans.z]
                self.cam_to_base_quat = [rot.x, rot.y, rot.z, rot.w]
                rospy.loginfo('从TF树获取外参成功: trans=%s, quat=%s',
                             self.cam_to_base_trans, self.cam_to_base_quat)
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                    tf2_ros.ExtrapolationException) as e:
                rospy.logerr('无法从TF树获取外参: %s，使用参数服务器默认值', e)
                self._load_extrinsic_from_params()
        else:
            # 从参数服务器读取
            self._load_extrinsic_from_params()

    def _load_extrinsic_from_params(self):
        """从参数服务器加载外参（默认值来自alicia_grasp.py的标定）"""
        cam_quat_default = [-0.65849396, -0.50618529, 0.29951161, 0.4695263]
        cam_trans_default = [0.89868963, -0.31994983, 0.37720613]
        self.cam_to_base_quat = self._load_vector_param(
            '~cam_to_base_quaternion', cam_quat_default
        )
        self.cam_to_base_trans = self._load_vector_param(
            '~cam_to_base_translation', cam_trans_default
        )
        rospy.loginfo('从参数服务器加载外参: trans=%s, quat=%s',
                     self.cam_to_base_trans, self.cam_to_base_quat)

    @staticmethod
    def _load_vector_param(param_name: str, default: List[float]) -> List[float]:
        value = rospy.get_param(param_name, default)
        if len(value) != len(default):
            rospy.logwarn('%s 长度不正确，使用默认值 %s', param_name, default)
            return default
        return value
    
    def _publish_camera_base_tf(self):
        """发布 base_link -> camera_color_optical_frame 的静态TF（从外参文件读取）"""
        try:
            # easy_handeye格式说明：
            # transformation 是从 tracking_base_frame (camera_color_optical_frame) 到 robot_base_frame (base_link) 的变换
            # 即：camera_color_optical_frame -> base_link
            # 但TF需要的是：base_link -> camera_color_optical_frame（需要取逆）
            
            rospy.loginfo('=' * 80)
            rospy.loginfo('加载外参数据用于TF发布:')
            rospy.loginfo('  外参文件: %s', self.extrinsic_calib_file)
            rospy.loginfo('  原始外参 (camera -> base):')
            rospy.loginfo('    位置: [%.6f, %.6f, %.6f]', 
                         self.cam_to_base_trans[0], self.cam_to_base_trans[1], self.cam_to_base_trans[2])
            rospy.loginfo('    四元数: [%.6f, %.6f, %.6f, %.6f]',
                         self.cam_to_base_quat[0], self.cam_to_base_quat[1], 
                         self.cam_to_base_quat[2], self.cam_to_base_quat[3])
            
            # 构建变换矩阵：camera_color_optical_frame -> base_link
            T_cam_base = quaternion_matrix(self.cam_to_base_quat)
            T_cam_base[0, 3] = self.cam_to_base_trans[0]
            T_cam_base[1, 3] = self.cam_to_base_trans[1]
            T_cam_base[2, 3] = self.cam_to_base_trans[2]
            
            # 计算逆变换：base_link -> camera_color_optical_frame
            T_base_cam = np.linalg.inv(T_cam_base)
            
            # 提取位置和姿态
            t_base_cam = T_base_cam[:3, 3]
            q_base_cam = quaternion_from_matrix(T_base_cam)
            
            # 创建TF消息：base_link -> camera_color_optical_frame
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = rospy.Time.now()
            transform_stamped.header.frame_id = self.base_frame_id  # base_link
            transform_stamped.child_frame_id = self.cam_frame_id  # camera_color_optical_frame
            
            transform_stamped.transform.translation.x = t_base_cam[0]
            transform_stamped.transform.translation.y = t_base_cam[1]
            transform_stamped.transform.translation.z = t_base_cam[2]
            transform_stamped.transform.rotation.x = q_base_cam[0]
            transform_stamped.transform.rotation.y = q_base_cam[1]
            transform_stamped.transform.rotation.z = q_base_cam[2]
            transform_stamped.transform.rotation.w = q_base_cam[3]
            
            # 发布静态TF
            self.static_tf_broadcaster.sendTransform(transform_stamped)
            rospy.loginfo('已发布静态TF: %s -> %s', self.base_frame_id, self.cam_frame_id)
            rospy.loginfo('  TF位置 (base -> camera): [%.6f, %.6f, %.6f]', 
                         t_base_cam[0], t_base_cam[1], t_base_cam[2])
            rospy.loginfo('  TF姿态 (base -> camera): [%.6f, %.6f, %.6f, %.6f]', 
                         q_base_cam[0], q_base_cam[1], q_base_cam[2], q_base_cam[3])
            rospy.loginfo('=' * 80)
        except Exception as e:
            rospy.logerr('发布相机外参TF失败: %s', e)
            import traceback
            rospy.logerr(traceback.format_exc())

    def _load_model(self, ckpt_path: str) -> GraspNet:
        net = GraspNet(
            input_feature_dim=0,
            num_view=300,
            num_angle=12,
            num_depth=4,
            is_training=False,
        )
        net.to(self.device)
        checkpoint = torch.load(ckpt_path, map_location=self.device)
        net.load_state_dict(checkpoint['model_state_dict'])
        rospy.loginfo(
            '加载 GraspNet checkpoint: %s (epoch=%d)',
            ckpt_path,
            checkpoint.get('epoch', -1),
        )
        return net
    
    def camera_to_base(self, t_camera2object, R_camera2object):
        """
        将相机坐标系下的物体位姿转换到基座坐标系。
        
        Args:
            t_camera2object: 物体在相机坐标系下的位置 (x, y, z)
            R_camera2object: 物体在相机坐标系下的旋转矩阵 (3x3)
            
        Returns:
            tuple: (t_base2object, q_base2object) 基座坐标系下的位置和四元数
        """
        # 创建物体在相机坐标系下的4x4变换矩阵
        T_camera2object = np.eye(4)
        T_camera2object[:3, :3] = R_camera2object
        T_camera2object[:3, 3] = t_camera2object
        
        # 使用已加载的外参标定数据（T_cam_base 是从 camera_color_optical_frame 到 base_link 的变换）
        # 但我们需要 T_base2cam（base -> camera）来进行正确的坐标转换
        # T_base2object = T_base2cam * T_camera2object
        # 而 T_base2cam = inv(T_cam_base)
        T_cam_base = self.T_cam_base.copy()
        T_base2cam = np.linalg.inv(T_cam_base)
        
        # 计算物体在基座坐标系下的位姿
        # T_base2object = T_base2cam * T_camera2object
        T_base2object = np.dot(T_base2cam, T_camera2object)
        
        # 应用坐标系转换（参考 alicia_grasp.py）
        T_base2object[:3, :3] = np.dot(T_base2object[:3, :3], np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]))
        T_base2object[:3, :3] = np.dot(T_base2object[:3, :3], np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]]))
        
        # 提取位置和姿态
        t_base2object = T_base2object[:3, 3]
        q_base2object = quaternion_from_matrix(T_base2object)
        
        return t_base2object, q_base2object
    
    def get_tool0_pose(self):
        """
        获取当前tool0的位姿（相对于base_link）
        
        Returns:
            tuple: (t_base2tool0, q_base2tool0) 如果成功，否则返回None
        """
        if self.tf_buffer is None:
            return None
        
        try:
            # 查找base_link到tool0的变换
            transform = self.tf_buffer.lookup_transform(
                self.base_frame_id, 'tool0', 
                rospy.Time(0), timeout=rospy.Duration(0.1)
            )
            trans = transform.transform.translation
            rot = transform.transform.rotation
            
            t_base2tool0 = np.array([trans.x, trans.y, trans.z])
            q_base2tool0 = np.array([rot.x, rot.y, rot.z, rot.w])
            
            return t_base2tool0, q_base2tool0
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logwarn('无法获取tool0位姿: %s', e)
            return None
    
    def publish_object_tf(self, t_base2object, q_base2object, frame_id='detected_object'):
        """
        发布检测到的物体相对于base_link的TF变换
        
        Args:
            t_base2object: 物体在base_link坐标系下的位置
            q_base2object: 物体在base_link坐标系下的四元数
            frame_id: TF frame名称
        """
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = rospy.Time.now()
        transform_stamped.header.frame_id = self.base_frame_id
        transform_stamped.child_frame_id = frame_id
        
        transform_stamped.transform.translation.x = t_base2object[0]
        transform_stamped.transform.translation.y = t_base2object[1]
        transform_stamped.transform.translation.z = t_base2object[2]
        
        transform_stamped.transform.rotation.x = q_base2object[0]
        transform_stamped.transform.rotation.y = q_base2object[1]
        transform_stamped.transform.rotation.z = q_base2object[2]
        transform_stamped.transform.rotation.w = q_base2object[3]
        
        # 发布TF变换
        self.tf_broadcaster.sendTransform(transform_stamped)

    def _apply_color_filter(self, points: np.ndarray, colors: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """应用颜色过滤，只保留目标颜色的点"""
        if not self.color_filter_enabled or len(points) == 0:
            return points, colors
        
        # 将RGB颜色（0-1范围）转换为0-255范围，然后转换为HSV
        colors_uint8 = (colors * 255).astype(np.uint8)
        # 注意：colors可能是RGB格式，需要转换为BGR用于OpenCV
        colors_bgr = colors_uint8[:, [2, 1, 0]]  # RGB -> BGR
        colors_bgr_reshaped = colors_bgr.reshape(-1, 1, 3)  # 需要reshape为(N, 1, 3)
        hsv_colors = cv2.cvtColor(colors_bgr_reshaped, cv2.COLOR_BGR2HSV).reshape(-1, 3)
        
        # 根据目标颜色创建掩码
        if self.target_color == 'red':
            # 红色在HSV中跨越0度，需要两个范围
            mask1 = np.all((hsv_colors >= self.color_ranges['red']['lower1']) & 
                          (hsv_colors <= self.color_ranges['red']['upper1']), axis=1)
            mask2 = np.all((hsv_colors >= self.color_ranges['red']['lower2']) & 
                          (hsv_colors <= self.color_ranges['red']['upper2']), axis=1)
            mask = mask1 | mask2
        else:
            color_range = self.color_ranges.get(self.target_color, self.color_ranges['red'])
            if 'lower' in color_range:
                mask = np.all((hsv_colors >= color_range['lower']) & 
                             (hsv_colors <= color_range['upper']), axis=1)
            else:
                # 如果没有找到对应的颜色范围，不进行过滤
                rospy.logwarn('未找到颜色 %s 的范围定义，跳过颜色过滤', self.target_color)
                return points, colors
        
        filtered_points = points[mask]
        filtered_colors = colors[mask]
        
        if len(filtered_points) < len(points):
            rospy.loginfo('颜色过滤 (%s): %d -> %d 点 (保留 %.1f%%)',
                         self.target_color, len(points), len(filtered_points),
                         100.0 * len(filtered_points) / len(points) if len(points) > 0 else 0)
        
        return filtered_points, filtered_colors

    def _apply_roi_filter(self, points: np.ndarray, colors: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """应用ROI裁剪，保留在3D边界框内的点"""
        if not self.roi_enabled or len(points) == 0:
            return points, colors
        
        mask = np.ones(len(points), dtype=bool)
        
        # X轴裁剪
        if self.roi_x_min is not None:
            mask &= (points[:, 0] >= self.roi_x_min)
        if self.roi_x_max is not None:
            mask &= (points[:, 0] <= self.roi_x_max)
        
        # Y轴裁剪
        if self.roi_y_min is not None:
            mask &= (points[:, 1] >= self.roi_y_min)
        if self.roi_y_max is not None:
            mask &= (points[:, 1] <= self.roi_y_max)
        
        # Z轴裁剪（深度）
        if self.roi_z_min is not None:
            mask &= (points[:, 2] >= self.roi_z_min)
        if self.roi_z_max is not None:
            mask &= (points[:, 2] <= self.roi_z_max)
        
        filtered_points = points[mask]
        filtered_colors = colors[mask]
        
        if len(filtered_points) < len(points):
            rospy.loginfo('ROI裁剪: %d -> %d 点 (保留 %.1f%%)',
                         len(points), len(filtered_points),
                         100.0 * len(filtered_points) / len(points) if len(points) > 0 else 0)
        
        return filtered_points, filtered_colors

    @staticmethod
    def _extract_rgb(rgb_float: float) -> Tuple[float, float, float]:
        """从 packed float 中解析 RGB，返回 0~1 范围"""
        rgb_uint = struct.unpack('I', struct.pack('f', rgb_float))[0]
        r = (rgb_uint >> 16) & 0xFF
        g = (rgb_uint >> 8) & 0xFF
        b = rgb_uint & 0xFF
        return r / 255.0, g / 255.0, b / 255.0

    def _pointcloud2_to_numpy(
        self, msg: PointCloud2
    ) -> Tuple[np.ndarray, np.ndarray]:
        fields = [f.name for f in msg.fields]
        has_rgb = 'rgb' in fields or 'rgba' in fields
        if has_rgb:
            field_names = ('x', 'y', 'z', 'rgb')
        else:
            field_names = ('x', 'y', 'z')

        points = []
        colors = []
        for pt in pc2.read_points(msg, field_names=field_names, skip_nans=True):
            x, y, z = pt[0], pt[1], pt[2]
            if not np.isfinite(x) or not np.isfinite(y) or not np.isfinite(z):
                continue
            points.append((x, y, z))
            if has_rgb:
                colors.append(self._extract_rgb(pt[3]))
        if not points:
            return (
                np.empty((0, 3), dtype=np.float32),
                np.empty((0, 3), dtype=np.float32),
            )

        points_np = np.asarray(points, dtype=np.float32)
        if has_rgb:
            colors_np = np.asarray(colors, dtype=np.float32)
        else:
            colors_np = np.zeros_like(points_np)

        # 应用ROI裁剪
        if self.roi_enabled:
            points_np, colors_np = self._apply_roi_filter(points_np, colors_np)
        
        # 应用颜色过滤（只保留目标颜色的点）
        if self.color_filter_enabled:
            points_np, colors_np = self._apply_color_filter(points_np, colors_np)

        return points_np, colors_np

    def _prepare_end_points(self, points: np.ndarray, colors: np.ndarray):
        if len(points) < 10:
            raise ValueError('点云有效点数量不足，无法进行推理')

        if len(points) >= self.num_point:
            idx = np.random.choice(len(points), self.num_point, replace=False)
        else:
            base = np.arange(len(points))
            extra = np.random.choice(
                len(points), self.num_point - len(points), replace=True
            )
            idx = np.concatenate([base, extra], axis=0)
        sampled_points = points[idx]
        sampled_colors = colors[idx]

        end_points = {}
        tensor_points = torch.from_numpy(
            sampled_points[np.newaxis, ...].astype(np.float32)
        ).to(self.device)
        end_points['point_clouds'] = tensor_points
        end_points['cloud_colors'] = sampled_colors.astype(np.float32)
        return end_points, sampled_points

    def pointcloud_callback(self, msg: PointCloud2):
        if self.processing:
            return
        self.processing = True
        try:
            points, colors = self._pointcloud2_to_numpy(msg)
            if points.size == 0:
                rospy.logwarn('收到空点云，跳过')
                return
            end_points, sampled_points = self._prepare_end_points(points, colors)

            with torch.no_grad():
                preds = self.net(end_points)
                grasp_preds = pred_decode(preds)

            gg_array = (
                grasp_preds[0].detach().cpu().numpy()
                if isinstance(grasp_preds[0], torch.Tensor)
                else grasp_preds[0]
            )
            gg = GraspGroup(gg_array)
            # 保存完整点云和颜色用于可视化
            self._report_grasps(gg, sampled_points, points, colors)
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logerr('GraspNet 推理失败: %s', exc)
        finally:
            self.processing = False

    def _report_grasps(self, gg: GraspGroup, sampled_points: np.ndarray,
                      full_points: np.ndarray = None, full_colors: np.ndarray = None):
        rospy.loginfo('当前点云采样点数: %d', sampled_points.shape[0])
        if len(gg) == 0:
            rospy.logwarn('GraspNet 未检测到抓取候选')
            return

        rospy.loginfo('原始检测到 %d 个抓取候选', len(gg))
        
        # NMS和排序
        gg.nms()
        gg.sort_by_score()
        
        # 只处理最佳的一个抓取候选
        if len(gg) == 0:
            rospy.logwarn('NMS后没有有效抓取候选')
            return
        
        best_grasp = gg[0]  # 得分最高的抓取
        
        # 只显示最佳抓取的信息
        rospy.loginfo('=' * 80)
        rospy.loginfo('检测到最佳抓取候选 (分数: %.4f)', best_grasp.score)
        rospy.loginfo('=' * 80)
        
        # 转换最佳抓取候选到base_link坐标系并添加到缓冲区（用于稳定化）
        if hasattr(self, 'T_cam_base'):
            t_camera2object = best_grasp.translation
            R_camera2object = best_grasp.rotation_matrix
            
            # 转换到base_link坐标系
            t_base2object, q_base2object = self.camera_to_base(t_camera2object, R_camera2object)
            
            # 添加到缓冲区（带异常值检测）
            with self.buffer_lock:
                # 异常值检测：如果缓冲区不为空，检查新值是否异常
                is_outlier = False
                if len(self.pose_buffer) > 0:
                    # 计算与缓冲区中最后一个值的位置差异
                    last_pos = self.pose_buffer[-1]['position']
                    pos_diff = np.linalg.norm(t_base2object - last_pos)
                    
                    # 计算姿态差异（使用四元数角度差）
                    last_quat = self.pose_buffer[-1]['quaternion']
                    # 四元数点积的绝对值（归一化后）
                    dot_product = abs(np.dot(q_base2object, last_quat))
                    dot_product = min(1.0, max(-1.0, dot_product))  # 限制在[-1, 1]
                    ori_diff = 2 * np.arccos(dot_product)  # 角度差（弧度）
                    
                    if pos_diff > self.max_position_change:
                        rospy.logwarn('检测到位置异常值，跳过: 位置变化=%.4f m (阈值=%.4f m)', 
                                     pos_diff, self.max_position_change)
                        is_outlier = True
                    elif ori_diff > self.max_orientation_change:
                        rospy.logwarn('检测到姿态异常值，跳过: 姿态变化=%.4f rad (阈值=%.4f rad)', 
                                     ori_diff, self.max_orientation_change)
                        is_outlier = True
                
                # 如果连续太多异常值，清空缓冲区重新开始（可能是物体移动了或检测不稳定）
                if is_outlier:
                    self.consecutive_outliers += 1
                    if self.consecutive_outliers >= self.max_consecutive_outliers:
                        rospy.logwarn('连续 %d 次检测到异常值，清空缓冲区重新开始', self.consecutive_outliers)
                        self.pose_buffer.clear()
                        self.consecutive_outliers = 0
                        rospy.loginfo('缓冲区已清空，等待新的检测结果...')
                
                if not is_outlier:
                    # 重置连续异常值计数
                    self.consecutive_outliers = 0
                    self.pose_buffer.append({
                        'position': t_base2object.copy(),
                        'quaternion': q_base2object.copy()
                    })
                    
                    buffer_count = len(self.pose_buffer)
                    rospy.loginfo('缓冲区: %d/%d', buffer_count, self.buffer_size)
                    
                    # 当缓冲区满时，计算均值并发送
                    if buffer_count >= self.buffer_size:
                        try:
                            # 计算位置均值（直接平均）
                            positions = np.array([p['position'] for p in self.pose_buffer])
                            mean_position = np.mean(positions, axis=0)
                            
                            # 计算姿态均值（通过旋转矩阵平均）
                            rotation_matrices = []
                            for p in self.pose_buffer:
                                q = p['quaternion']
                                T = quaternion_matrix(q)
                                rotation_matrices.append(T[:3, :3])
                            
                            rotation_matrices = np.array(rotation_matrices)
                            
                            # 对旋转矩阵求平均（使用Frobenius范数）
                            mean_rotation_matrix = np.mean(rotation_matrices, axis=0)
                            # 正交化（SVD分解）
                            U, _, Vt = np.linalg.svd(mean_rotation_matrix)
                            mean_rotation_matrix = U @ Vt
                            # 确保是旋转矩阵（行列式为1）
                            if np.linalg.det(mean_rotation_matrix) < 0:
                                U[:, -1] *= -1
                                mean_rotation_matrix = U @ Vt
                            
                            # 转换回四元数（需要4x4矩阵）
                            T_mean = np.eye(4)
                            T_mean[:3, :3] = mean_rotation_matrix
                            mean_quaternion = quaternion_from_matrix(T_mean)
                            
                            # 清空缓冲区（在发布之前清空，避免重复处理）
                            self.pose_buffer.clear()
                            self.consecutive_outliers = 0  # 重置连续异常值计数
                            
                            # 发布均值结果
                            rospy.loginfo('=' * 80)
                            rospy.loginfo('缓冲区已满，计算均值并发送（%d个结果）', self.buffer_size)
                            rospy.loginfo('=' * 80)
                            
                            # 获取当前tool0位姿（用于对比）
                            tool0_pose = self.get_tool0_pose()
                            
                            # 发布TF变换（detected_object -> base_link）
                            self.publish_object_tf(mean_position, mean_quaternion, frame_id='detected_object')
                            
                            # 发布PoseStamped消息到话题（z坐标增加0.1米）
                            pose_msg = PoseStamped()
                            pose_msg.header.stamp = rospy.Time.now()
                            pose_msg.header.frame_id = self.base_frame_id
                            pose_msg.pose.position.x = mean_position[0]
                            pose_msg.pose.position.y = mean_position[1]
                            pose_msg.pose.position.z = mean_position[2] + 0.1  # z坐标增加0.1米
                            pose_msg.pose.orientation.x = mean_quaternion[0]
                            pose_msg.pose.orientation.y = mean_quaternion[1]
                            pose_msg.pose.orientation.z = mean_quaternion[2]
                            pose_msg.pose.orientation.w = mean_quaternion[3]
                            self.pose_pub.publish(pose_msg)
                            
                            # 输出转换后的位置信息
                            euler_angles = euler_from_quaternion(mean_quaternion)
                            euler_angles_deg = np.degrees(euler_angles)
                            
                            rospy.loginfo('【检测到的物块位姿 - base_link坐标系】')
                            rospy.loginfo('  原始位置 (x, y, z): [%.6f, %.6f, %.6f] m', 
                                         mean_position[0], mean_position[1], mean_position[2])
                            rospy.loginfo('  发布位置 (x, y, z): [%.6f, %.6f, %.6f] m (z已增加0.1m)', 
                                         mean_position[0], mean_position[1], mean_position[2] + 0.1)
                            rospy.loginfo('  四元数 (qx, qy, qz, qw): [%.6f, %.6f, %.6f, %.6f]',
                                         mean_quaternion[0], mean_quaternion[1], mean_quaternion[2], mean_quaternion[3])
                            rospy.loginfo('  欧拉角 (Roll, Pitch, Yaw): [%.2f°, %.2f°, %.2f°]',
                                         euler_angles_deg[0], euler_angles_deg[1], euler_angles_deg[2])
                            
                            # 输出tool0位姿（用于对比）
                            if tool0_pose is not None:
                                t_tool0, q_tool0 = tool0_pose
                                euler_tool0 = euler_from_quaternion(q_tool0)
                                euler_tool0_deg = np.degrees(euler_tool0)
                                
                                rospy.loginfo('【当前tool0位姿 - base_link坐标系】')
                                rospy.loginfo('  位置 (x, y, z): [%.6f, %.6f, %.6f] m', 
                                             t_tool0[0], t_tool0[1], t_tool0[2])
                                rospy.loginfo('  四元数 (qx, qy, qz, qw): [%.6f, %.6f, %.6f, %.6f]',
                                             q_tool0[0], q_tool0[1], q_tool0[2], q_tool0[3])
                                rospy.loginfo('  欧拉角 (Roll, Pitch, Yaw): [%.2f°, %.2f°, %.2f°]',
                                             euler_tool0_deg[0], euler_tool0_deg[1], euler_tool0_deg[2])
                                
                                # 计算位置差异
                                pos_diff = np.linalg.norm(mean_position - t_tool0)
                                rospy.loginfo('【位姿差异】')
                                rospy.loginfo('  位置差异: %.6f m', pos_diff)
                                rospy.loginfo('  位置差异向量 (物块 - tool0): [%.6f, %.6f, %.6f] m',
                                             mean_position[0] - t_tool0[0],
                                             mean_position[1] - t_tool0[1],
                                             mean_position[2] - t_tool0[2])
                            else:
                                rospy.logwarn('无法获取tool0位姿，跳过对比')
                            
                            rospy.loginfo('  TF frame: detected_object (相对于 base_link)')
                            rospy.loginfo('  话题: %s', self.pose_pub.resolved_name)
                            rospy.loginfo('=' * 80)
                        except Exception as e:
                            rospy.logerr('计算均值失败: %s', e)
                            import traceback
                            rospy.logerr(traceback.format_exc())
                            # 即使出错也要清空缓冲区，避免一直累积
                            self.pose_buffer.clear()
                            rospy.logwarn('已清空缓冲区，等待下次检测')
                else:
                    rospy.loginfo('跳过异常值，继续收集数据...')
        else:
            rospy.logwarn('外参标定数据未加载，无法转换坐标系')
        
        # 可视化抓取结果（只显示最佳的一个）
        if self.visualize:
            best_grasp_group = GraspGroup(best_grasp.grasp_array.reshape(1, -1))
            self._visualize_grasps(best_grasp_group, sampled_points, full_points, full_colors)
        
        # 在RGB图像上可视化抓取结果（只显示最佳的一个）
        if self.visualize_on_rgb:
            best_grasp_group = GraspGroup(best_grasp.grasp_array.reshape(1, -1))
            self._visualize_grasps_on_rgb(best_grasp_group)

    def _log_top_grasps(self, gg: GraspGroup):
        top_k = min(self.top_k, len(gg))
        rospy.loginfo('=' * 80)
        rospy.loginfo('检测到 %d 个抓取，展示前 %d 个：', len(gg), top_k)
        rospy.loginfo('=' * 80)
        
        for i in range(top_k):
            grasp = gg[i]
            center = grasp.translation
            rotation_matrix = grasp.rotation_matrix
            
            # 将旋转矩阵转换为四元数
            T_grasp = np.eye(4)
            T_grasp[:3, :3] = rotation_matrix
            T_grasp[:3, 3] = center
            quaternion = quaternion_from_matrix(T_grasp)
            
            # 将旋转矩阵转换为欧拉角（RPY，弧度）
            euler_rad = euler_from_matrix(rotation_matrix, 'sxyz')
            euler_deg = np.degrees(euler_rad)
            
            # 输出基本信息
            rospy.loginfo('-' * 80)
            rospy.loginfo('抓取候选 #%d (分数: %.4f)', i, grasp.score)
            rospy.loginfo('  抓取参数: width=%.4f m, height=%.4f m, depth=%.4f m',
                         grasp.width, grasp.height, grasp.depth)
            
            # 输出相机坐标系下的位姿
            rospy.loginfo('  【相机坐标系 (camera_color_optical_frame)】')
            rospy.loginfo('    位置 (x, y, z): [%.6f, %.6f, %.6f] m', 
                         center[0], center[1], center[2])
            rospy.loginfo('    四元数 (qx, qy, qz, qw): [%.6f, %.6f, %.6f, %.6f]',
                         quaternion[0], quaternion[1], quaternion[2], quaternion[3])
            rospy.loginfo('    欧拉角 (Roll, Pitch, Yaw): [%.2f°, %.2f°, %.2f°]',
                         euler_deg[0], euler_deg[1], euler_deg[2])
            rospy.loginfo('    旋转矩阵:')
            rospy.loginfo('      [%.6f, %.6f, %.6f]', 
                         rotation_matrix[0, 0], rotation_matrix[0, 1], rotation_matrix[0, 2])
            rospy.loginfo('      [%.6f, %.6f, %.6f]',
                         rotation_matrix[1, 0], rotation_matrix[1, 1], rotation_matrix[1, 2])
            rospy.loginfo('      [%.6f, %.6f, %.6f]',
                         rotation_matrix[2, 0], rotation_matrix[2, 1], rotation_matrix[2, 2])
            
            # 转换到基座坐标系（如果外参可用）
            T_base_grasp = None
            if hasattr(self, 'T_cam_base'):
                try:
                    # 构建抓取位姿的4x4变换矩阵（相机坐标系）
                    T_cam_grasp = np.eye(4)
                    T_cam_grasp[:3, :3] = rotation_matrix
                    T_cam_grasp[:3, 3] = center
                    
                    # 转换到基座坐标系
                    # T_base_grasp = T_cam_base * T_cam_grasp
                    T_base_grasp = np.dot(self.T_cam_base, T_cam_grasp)
                    
                    # 提取位置和姿态
                    center_base = T_base_grasp[:3, 3]
                    rotation_base = T_base_grasp[:3, :3]
                    quaternion_base = quaternion_from_matrix(T_base_grasp)
                    euler_base_rad = euler_from_matrix(rotation_base, 'sxyz')
                    euler_base_deg = np.degrees(euler_base_rad)
                    
                    rospy.loginfo('  【基座坐标系 (base_link)】')
                    rospy.loginfo('    位置 (x, y, z): [%.6f, %.6f, %.6f] m',
                                 center_base[0], center_base[1], center_base[2])
                    rospy.loginfo('    四元数 (qx, qy, qz, qw): [%.6f, %.6f, %.6f, %.6f]',
                                 quaternion_base[0], quaternion_base[1], 
                                 quaternion_base[2], quaternion_base[3])
                    rospy.loginfo('    欧拉角 (Roll, Pitch, Yaw): [%.2f°, %.2f°, %.2f°]',
                                 euler_base_deg[0], euler_base_deg[1], euler_base_deg[2])
                except Exception as e:
                    rospy.logwarn('    转换到基座坐标系失败: %s', e)
            
            # 转换到机械臂坐标系（如果TF可用）
            if T_base_grasp is not None and self.use_tf_for_arm_transform and self.tf_buffer is not None:
                try:
                    # 查找从base_link到机械臂坐标系的变换
                    rospy.sleep(0.1)  # 等待TF更新
                    transform = self.tf_buffer.lookup_transform(
                        self.base_frame_id, self.arm_frame_id, 
                        rospy.Time(0), timeout=rospy.Duration(1.0)
                    )
                    
                    # 构建变换矩阵
                    trans = transform.transform.translation
                    rot = transform.transform.rotation
                    T_base_arm = quaternion_matrix([rot.x, rot.y, rot.z, rot.w])
                    T_base_arm[0, 3] = trans.x
                    T_base_arm[1, 3] = trans.y
                    T_base_arm[2, 3] = trans.z
                    
                    # 计算机械臂坐标系到base_link的逆变换
                    T_arm_base = np.linalg.inv(T_base_arm)
                    
                    # 将抓取位姿转换到机械臂坐标系
                    T_arm_grasp = np.dot(T_arm_base, T_base_grasp)
                    
                    # 提取位置和姿态
                    center_arm = T_arm_grasp[:3, 3]
                    rotation_arm = T_arm_grasp[:3, :3]
                    quaternion_arm = quaternion_from_matrix(T_arm_grasp)
                    euler_arm_rad = euler_from_matrix(rotation_arm, 'sxyz')
                    euler_arm_deg = np.degrees(euler_arm_rad)
                    
                    rospy.loginfo('  【机械臂坐标系 (%s) - 相对位姿】', self.arm_frame_id)
                    rospy.loginfo('    位置 (x, y, z): [%.6f, %.6f, %.6f] m',
                                 center_arm[0], center_arm[1], center_arm[2])
                    rospy.loginfo('    四元数 (qx, qy, qz, qw): [%.6f, %.6f, %.6f, %.6f]',
                                 quaternion_arm[0], quaternion_arm[1], 
                                 quaternion_arm[2], quaternion_arm[3])
                    rospy.loginfo('    欧拉角 (Roll, Pitch, Yaw): [%.2f°, %.2f°, %.2f°]',
                                 euler_arm_deg[0], euler_arm_deg[1], euler_arm_deg[2])
                    rospy.loginfo('    旋转矩阵:')
                    rospy.loginfo('      [%.6f, %.6f, %.6f]',
                                 rotation_arm[0, 0], rotation_arm[0, 1], rotation_arm[0, 2])
                    rospy.loginfo('      [%.6f, %.6f, %.6f]',
                                 rotation_arm[1, 0], rotation_arm[1, 1], rotation_arm[1, 2])
                    rospy.loginfo('      [%.6f, %.6f, %.6f]',
                                 rotation_arm[2, 0], rotation_arm[2, 1], rotation_arm[2, 2])
                    
                    # 计算相对距离和角度
                    distance = np.linalg.norm(center_arm)
                    rospy.loginfo('    相对距离: %.6f m', distance)
                    
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException) as e:
                    rospy.logwarn('    无法从TF树获取机械臂坐标系变换 (%s -> %s): %s',
                                 self.base_frame_id, self.arm_frame_id, e)
                except Exception as e:
                    rospy.logwarn('    转换到机械臂坐标系失败: %s', e)
            
            rospy.loginfo('-' * 80)
        
        rospy.loginfo('=' * 80)

    def rgb_image_callback(self, msg: Image):
        """RGB图像回调，保存最新图像用于可视化"""
        try:
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
            with self.rgb_image_lock:
                self.rgb_image = cv_image.copy()
        except Exception as e:
            # 静默处理 GDAL 库版本冲突的警告（不影响功能）
            error_msg = str(e)
            if 'libgdal' in error_msg or 'TIFFReadRGBATileExt' in error_msg:
                # GDAL 库版本冲突，静默处理
                pass
            else:
                # 其他错误仍然记录
                rospy.logwarn('RGB图像转换失败: %s', e)
    
    def _project_3d_to_2d(self, point_3d: np.ndarray) -> Tuple[int, int]:
        """将3D点投影到2D图像坐标"""
        if self.camera_intrinsic is None:
            return None, None
        
        fx = self.camera_intrinsic['fx']
        fy = self.camera_intrinsic['fy']
        cx = self.camera_intrinsic['cx']
        cy = self.camera_intrinsic['cy']
        
        x, y, z = point_3d[0], point_3d[1], point_3d[2]
        
        if z <= 0:
            return None, None
        
        u = int(fx * x / z + cx)
        v = int(fy * y / z + cy)
        
        return u, v
    
    def _visualize_grasps_on_rgb(self, gg: GraspGroup):
        """在RGB图像上可视化抓取结果"""
        if len(gg) == 0:
            return
        
        with self.rgb_image_lock:
            if self.rgb_image is None:
                return
            vis_image = self.rgb_image.copy()
        
        # 获取前N个抓取进行可视化
        visualize_k = min(self.visualize_top_k, len(gg))
        
        # 定义颜色（BGR格式，OpenCV使用）
        colors_bgr = [
            (0, 0, 255),    # 红色 - 最佳
            (0, 255, 0),    # 绿色 - 第二
            (255, 0, 0),    # 蓝色 - 第三
            (0, 255, 255),  # 黄色 - 第四
            (255, 0, 255),  # 紫色 - 第五
        ]
        
        for i in range(visualize_k):
            grasp = gg[i]
            color = colors_bgr[i % len(colors_bgr)]
            
            # 获取抓取中心点
            center_3d = grasp.translation
            u, v = self._project_3d_to_2d(center_3d)
            
            if u is None or v is None:
                continue
            
            # 检查点是否在图像范围内
            h, w = vis_image.shape[:2]
            if u < 0 or u >= w or v < 0 or v >= h:
                continue
            
            # 绘制抓取中心点
            cv2.circle(vis_image, (u, v), 8, color, -1)
            cv2.circle(vis_image, (u, v), 12, color, 2)
            
            # 绘制抓取方向（从旋转矩阵获取）
            rotation_matrix = grasp.rotation_matrix
            # 抓取方向通常是Z轴（接近方向）
            approach_dir = rotation_matrix[:, 2]  # Z轴方向
            # 抓取宽度方向（通常是Y轴）
            width_dir = rotation_matrix[:, 1]  # Y轴方向
            
            # 计算抓取框的4个角点（在3D空间中）
            width = grasp.width
            depth = grasp.depth
            
            # 抓取框的4个角点（相对于中心）
            half_width = width / 2.0
            half_depth = depth / 2.0
            
            corners_3d = np.array([
                center_3d + half_width * width_dir + half_depth * approach_dir,  # 右上
                center_3d - half_width * width_dir + half_depth * approach_dir,  # 左上
                center_3d - half_width * width_dir - half_depth * approach_dir,  # 左下
                center_3d + half_width * width_dir - half_depth * approach_dir,  # 右下
            ])
            
            # 投影到2D
            corners_2d = []
            for corner_3d in corners_3d:
                u_c, v_c = self._project_3d_to_2d(corner_3d)
                if u_c is not None and v_c is not None:
                    corners_2d.append((u_c, v_c))
            
            # 绘制抓取框（如果所有角点都有效）
            if len(corners_2d) == 4:
                pts = np.array(corners_2d, dtype=np.int32)
                cv2.polylines(vis_image, [pts], True, color, 2)
                # 绘制对角线
                cv2.line(vis_image, tuple(corners_2d[0]), tuple(corners_2d[2]), color, 1)
                cv2.line(vis_image, tuple(corners_2d[1]), tuple(corners_2d[3]), color, 1)
            
            # 绘制抓取方向箭头
            end_point_3d = center_3d + 0.05 * approach_dir  # 5cm的箭头
            u_end, v_end = self._project_3d_to_2d(end_point_3d)
            if u_end is not None and v_end is not None:
                cv2.arrowedLine(vis_image, (u, v), (u_end, v_end), color, 2, tipLength=0.3)
            
            # 添加分数标签
            score_text = f'#{i+1}: {grasp.score:.2f}'
            cv2.putText(vis_image, score_text, (u + 15, v - 15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 发布可视化图像
        if self.publish_visualization:
            try:
                vis_msg = self.cv_bridge.cv2_to_imgmsg(vis_image, 'bgr8')
                self.pub_vis_image.publish(vis_msg)
            except Exception as e:
                rospy.logwarn('发布可视化图像失败: %s', e)
        
        # 显示图像（可选，用于调试）
        if self.visualize:
            cv2.imshow('GraspNet Visualization', vis_image)
            cv2.waitKey(1)  # 非阻塞，只更新显示

    def _visualize_grasps(self, gg: GraspGroup, sampled_points: np.ndarray,
                         full_points: np.ndarray = None, full_colors: np.ndarray = None):
        """可视化抓取结果：显示点云和抓取姿态"""
        if self.visualize_once and self.visualization_shown:
            return
        try:
            # 使用完整点云或采样点云
            if full_points is not None and len(full_points) > 0:
                display_points = full_points
                display_colors = full_colors if full_colors is not None else np.ones_like(full_points) * 0.5
            else:
                display_points = sampled_points
                display_colors = np.ones_like(sampled_points) * 0.5
            
            # 创建Open3D点云
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(display_points.astype(np.float32))
            pcd.colors = o3d.utility.Vector3dVector(display_colors.astype(np.float32))
            
            # 获取前N个抓取姿态进行可视化
            visualize_k = min(self.visualize_top_k, len(gg))
            rospy.loginfo('可视化前 %d 个抓取候选...', visualize_k)
            
            # 将抓取姿态转换为Open3D几何体
            grippers = gg[:visualize_k].to_open3d_geometry_list()
            
            # 为不同抓取设置不同颜色
            colors = [
                [1, 0, 0],  # 红色 - 最佳
                [0, 1, 0],  # 绿色 - 第二
                [0, 0, 1],  # 蓝色 - 第三
                [1, 1, 0],  # 黄色 - 第四
                [1, 0, 1],  # 紫色 - 第五
            ]
            for i, gripper in enumerate(grippers):
                color = colors[i % len(colors)]
                if isinstance(gripper, list):
                    for g in gripper:
                        g.paint_uniform_color(color)
                else:
                    gripper.paint_uniform_color(color)
            
            # 准备可视化列表
            vis_list = [pcd]
            for gripper in grippers:
                if isinstance(gripper, list):
                    vis_list.extend(gripper)
                else:
                    vis_list.append(gripper)
            
            # 显示可视化窗口
            rospy.loginfo('打开可视化窗口，按 Q 或关闭窗口继续...')
            o3d.visualization.draw_geometries(
                vis_list,
                window_name=f'GraspNet 检测结果 (前{visualize_k}个抓取)',
                width=1280,
                height=720
            )
            rospy.loginfo('可视化窗口已关闭')
            self.visualization_shown = True
        except Exception as exc:  # pylint: disable=broad-except
            rospy.logerr('可视化失败: %s', exc)

    def _publish_camera_tf_continuously(self):
        """持续发布 base_link -> camera_color_optical_frame 的TF（10Hz）"""
        rate = rospy.Rate(10.0)  # 10Hz
        while not rospy.is_shutdown():
            try:
                # 构建变换矩阵：camera_color_optical_frame -> base_link
                T_cam_base = quaternion_matrix(self.cam_to_base_quat)
                T_cam_base[0, 3] = self.cam_to_base_trans[0]
                T_cam_base[1, 3] = self.cam_to_base_trans[1]
                T_cam_base[2, 3] = self.cam_to_base_trans[2]
                
                # 计算逆变换：base_link -> camera_color_optical_frame
                T_base_cam = np.linalg.inv(T_cam_base)
                
                # 提取位置和姿态
                t_base_cam = T_base_cam[:3, 3]
                q_base_cam = quaternion_from_matrix(T_base_cam)
                
                # 创建TF消息：base_link -> camera_color_optical_frame
                transform_stamped = TransformStamped()
                transform_stamped.header.stamp = rospy.Time.now()
                transform_stamped.header.frame_id = self.base_frame_id  # base_link
                transform_stamped.child_frame_id = self.cam_frame_id  # camera_color_optical_frame
                
                transform_stamped.transform.translation.x = t_base_cam[0]
                transform_stamped.transform.translation.y = t_base_cam[1]
                transform_stamped.transform.translation.z = t_base_cam[2]
                transform_stamped.transform.rotation.x = q_base_cam[0]
                transform_stamped.transform.rotation.y = q_base_cam[1]
                transform_stamped.transform.rotation.z = q_base_cam[2]
                transform_stamped.transform.rotation.w = q_base_cam[3]
                
                # 发布TF
                self.tf_broadcaster.sendTransform(transform_stamped)
                rate.sleep()
            except Exception as e:
                rospy.logwarn('发布相机TF时出错: %s', e)
                rate.sleep()
    
    def spin(self):
        # 启动持续发布相机TF的线程
        import threading
        tf_thread = threading.Thread(target=self._publish_camera_tf_continuously)
        tf_thread.daemon = True
        tf_thread.start()
        rospy.loginfo('已启动持续发布相机TF的线程')
        rospy.spin()


def main():
    rospy.init_node('graspnet_pointcloud_node', anonymous=False)
    node = GraspNetInferenceNode()
    node.spin()


if __name__ == '__main__':
    main()

