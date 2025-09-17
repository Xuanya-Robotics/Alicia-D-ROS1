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
ROOT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.append(ROOT_DIR)
sys.path.append(os.path.join(ROOT_DIR, 'utils'))
sys.path.append(os.path.join(ROOT_DIR, 'pointnet2'))
from models.graspnet import GraspNet, pred_decode
from data_utils import CameraInfo, create_point_cloud_from_depth_image
from geometry_msgs.msg import PoseStamped
from tf.transformations import (
    quaternion_matrix,
    quaternion_from_matrix,
    euler_from_quaternion
)
import tf2_ros
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_matrix, quaternion_from_matrix, euler_from_quaternion


ros_path = '/opt/ros/noetic/lib/python3/dist-packages'  # Adjust this path if necessary


sys.path.remove(ros_path)  # Remove ROS path after importing cv_bridge


from cv_bridge import CvBridge

dataset_path = './graspness_implementation/'
robot_path = '/home/ubuntu/alicia_ws/src/alicia_d_moveit/scripts'
sys.path.append(robot_path)
sys.path.append(dataset_path)
from dataset.graspnet_dataset import minkowski_collate_fn
from moveit_control import MoveItRobotController

parser = argparse.ArgumentParser()
parser.add_argument('--checkpoint_path', default='np15000_graspness1e-1_bs4_lr1e-3_viewres_dataaug_fps_14D_epoch10.tar')


parser.add_argument('--seed_feat_dim', default=512, type=int, help='Point wise feature dim')
parser.add_argument('--camera', default='realsense', help='Camera split [realsense/kinect]')
parser.add_argument('--num_point', type=int, default=50000, help='Point Number [default: 15000]')
parser.add_argument('--batch_size', type=int, default=1, help='Batch Size during inference [default: 1]')
parser.add_argument('--voxel_size', type=float, default=0.005, help='Voxel Size for sparse convolution')
parser.add_argument('--collision_thresh', type=float, default=0.00,
                    help='Collision Threshold in collision detection [default: 0.01]')
parser.add_argument('--voxel_size_cd', type=float, default=0.01, help='Voxel Size for collision detection')
cfgs = parser.parse_args()


# Modified function signature to accept tf_buffer
def camera2base(t_optical2object, R_optical2object):
    """
    Calculates the target pose for the robot's end-effector (Link06) in base_link.

    Args:
        t_optical2object: Translation vector (x, y, z) of object in camera_color_optical_frame.
        R_optical2object: Rotation matrix (3x3) of object in camera_color_optical_frame.
        tf_buffer: An instance of tf2_ros.Buffer for TF lookups.

    Returns:
        A PoseStamped message for the target end-effector pose, or None on error.
    """
    tf_buffer = tf2_ros.Buffer()
    rospy.loginfo("Calculating target grasp pose...")

    # --- 1. Create Object Pose in Optical Frame ---
    T_optical2object = np.eye(4)
    T_optical2object[:3, :3] = R_optical2object
    T_optical2object[:3, 3] = t_optical2object
    rospy.loginfo(f"Input T_optical2object:\n{T_optical2object}")

    T_link2object = T_optical2object

    # --- 4. Load Camera to Base Transformation ---
    # Make sure these values are correct for base_link -> camera_link
    calib_qx, calib_qy, calib_qz, calib_qw = -0.6584939606665077,  -0.5061852896857801,  0.2995116119056745,  0.46952630448689786
    calib_tx, calib_ty, calib_tz = 0.898689629778175, -0.31994982737661337, 0.3772061264518505

    T_base2cam_link = quaternion_matrix([calib_qx, calib_qy, calib_qz, calib_qw])
    T_base2cam_link[0, 3] = calib_tx
    T_base2cam_link[1, 3] = calib_ty
    T_base2cam_link[2, 3] = calib_tz

    # --- 5. Calculate Object Pose in Base Frame ---
    T_base2object = np.dot(T_base2cam_link, T_link2object)
    T_base2object[:3, :3] = np.dot(T_base2object[:3, :3], np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]]))  # 注释掉这一行
    T_base2object[:3, :3] = np.dot(T_base2object[:3, :3], np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]])) # 注释掉这一行


    # --- 6. Extract Position and Orientation ---
    t_base2object = T_base2object[:3, 3]
    q_base2object = quaternion_from_matrix(T_base2object)
        
    

    # Calculate Euler angles for easier understanding
    euler_angles = euler_from_quaternion(q_base2object)
    euler_angles_deg = np.degrees(euler_angles)
    
    rospy.loginfo("Final Target Pose in base_link:")
    rospy.loginfo(f"  Position (x,y,z): {t_base2object}")
    rospy.loginfo(f"  Orientation (qx,qy,qz,qw): {q_base2object}")
    rospy.loginfo(f"  Orientation (Euler RPY deg): {euler_angles_deg}")
    

    # --- 8. Create PoseStamped Message ---
    pose_msg = PoseStamped()
    pose_msg.header.frame_id = "base_link"
    pose_msg.header.stamp = rospy.Time.now()

    pose_msg.pose.position.x = t_base2object[0]
    pose_msg.pose.position.y = t_base2object[1]
    pose_msg.pose.position.z = t_base2object[2]

    pose_msg.pose.orientation.x = q_base2object[0]
    pose_msg.pose.orientation.y = q_base2object[1]
    pose_msg.pose.orientation.z = q_base2object[2]
    pose_msg.pose.orientation.w = q_base2object[3]
    # Create static transform broadcaster for one-time publishing
    # Create static transform broadcaster for one-time publishing
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()
    
    # Create transform message
    transform_stamped = TransformStamped()
    transform_stamped.header.stamp = rospy.Time.now()
    transform_stamped.header.frame_id = "base_link"
    transform_stamped.child_frame_id = "grasp_frame"
    
    transform_stamped.transform.translation.x = t_base2object[0]
    transform_stamped.transform.translation.y = t_base2object[1]
    transform_stamped.transform.translation.z = t_base2object[2]
    
    transform_stamped.transform.rotation.x = q_base2object[0]
    transform_stamped.transform.rotation.y = q_base2object[1]
    transform_stamped.transform.rotation.z = q_base2object[2]
    transform_stamped.transform.rotation.w = q_base2object[3]
    
    # Publish the transform
    static_broadcaster.sendTransform(transform_stamped)
    # Also use a regular broadcaster for continuous publishing in a separate thread
    def publish_tf_continuously():
        br = tf2_ros.TransformBroadcaster()
        rate = rospy.Rate(10.0)  # 10Hz
        count = 0
        while count < 50 and not rospy.is_shutdown():  # Publish for ~5 seconds
            transform_stamped.header.stamp = rospy.Time.now()
            br.sendTransform(transform_stamped)
            rate.sleep()
            count += 1
    
    # Start continuous publishing in a separate thread
    import threading
    tf_thread = threading.Thread(target=publish_tf_continuously)
    tf_thread.daemon = True
    tf_thread.start()
    # Return the transformed pose for further use
    return t_base2object, q_base2object



def select_workspace(image, points=None):
    """
    Generate workspace mask based on provided or default points.
    :param image: Input image
    :param points: List of (x,y) points defining workspace corners. If None, use defaults.
    :return: Workspace mask
    """
    # Use default points if none provided
    if points is None:
        # Default points
        points = [[210, 6], [576, 419], [188, 434], [579, 7]]
    
    ordered_points = np.array(points, dtype=np.float32)

    # Order points clockwise
    def order_points_clockwise(pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]  # Top-left
        rect[2] = pts[np.argmax(s)]  # Bottom-right
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)]  # Top-right
        rect[3] = pts[np.argmax(diff)]  # Bottom-left
        return rect

    ordered_points = order_points_clockwise(ordered_points)
    
    # Create mask
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [np.array(ordered_points, dtype=np.int32)], 255)
    
    return mask


def show_image_and_get_pixel(image_to_show, window_name="Click to get pixel coordinates"):
    """
    Displays an image and prints the coordinates of left mouse clicks.
    Press 'Esc' to close the window.
    Returns a list of clicked points as (x, y) tuples.
    """
    clicked_points = []
    
    def mouse_callback(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # OpenCV coordinates are (x=column, y=row)
            print(f"Pixel coordinates: x={x}, y={y}")
            if y < image_to_show.shape[0] and x < image_to_show.shape[1]:
                # Show the pixel value at the clicked point
                if len(image_to_show.shape) == 3:  # Color image
                    pixel_value = image_to_show[y, x]
                    print(f"RGB value: {pixel_value}")
                else:  # Grayscale/depth image
                    pixel_value = image_to_show[y, x]
                    print(f"Pixel value: {pixel_value}")
                
                clicked_points.append((x, y))  # Store as (x, y)
            else:
                print("Clicked outside valid image area")

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, mouse_callback)

    print(f"Displaying image in window '{window_name}'.")
    print(f"Image dimensions: {image_to_show.shape[1]}x{image_to_show.shape[0]} (width x height)")
    print("Click on points to get coordinates. Press 'Esc' to continue.")
    
    while True:
        # Draw circles on the clicked points
        display_img = image_to_show.copy()
        for i, (px, py) in enumerate(clicked_points):
            cv2.circle(display_img, (px, py), 5, (0, 255, 0), 2)
            cv2.putText(display_img, f"{i+1}: ({px},{py})", (px+10, py), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        cv2.imshow(window_name, display_img)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:  # ESC key
            break
    
    cv2.destroyWindow(window_name)
    print(f"Selected {len(clicked_points)} points: {clicked_points}")
    return clicked_points  # Return the list of points

def data_process():    
    # ROS 桥接器，用于将 ROS 图像消息转换为 OpenCV 格式
    bridge = CvBridge()
    rospy.init_node('moveit_control_server', anonymous=False)
    # 订阅 RealSense 相机的相关话题depth_camera_info_topic
    color_topic = "/camera/color/image_raw"  # 彩色图像话题
    # depth_topic = "/camera/aligned_depth_to_color/image_raw"  # 深度图像话题
    depth_topic = "/camera/depth/image_raw"  # 深度图像话题
    color_camera_info_topic = "/camera/color/camera_info"  # 彩色相机信息话题
    depth_camera_info_topic = "/camera/depth/camera_info"  # 深度相机信息话题
    # depth_camera_info_topic = "/camera/aligned_depth_to_color/camera_info"  # 深度相机信息话题
    print("等待相机数据...")
    # 将 ROS 消息转换为 OpenCV 格式
    color_msg = rospy.wait_for_message(color_topic, Image)
    print("获取到相机数据。")
    depth_msg = rospy.wait_for_message(depth_topic, Image)
    print("获取到相机数据。")
    # 归一化并预处理图像
    rgb = bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8")  # 转换彩色图像
    depth = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough").astype(np.float32)  # 转换深度图像

    print("rgb.shape:", rgb.shape)

    # 将颜色值归一化到 [0, 1]
    color = rgb / 255.0
    color = color[:, :, ::-1]  # 将 RGB 转换为 BGR 格式

    # Use the function to get clicked points
    clicked_points = show_image_and_get_pixel(color.copy(), "Select workspace corners")
    
    # If points were selected, use them for workspace mask
    if len(clicked_points) >= 4:
        # Use only the first 4 points
        points = clicked_points[:4]
        # Update your select_workspace function to use these points
        workspace_mask = select_workspace(color, points)
    else:
        # Use default points
        workspace_mask = select_workspace(color)

    # 获取相机的内参信息
    depth_camera_info = rospy.wait_for_message(color_camera_info_topic, ROSCameraInfo)
    
    # 解析深度相机的内参矩阵
    intrinsic = np.array(depth_camera_info.K).reshape(3, 3)
    factor_depth = 1.0 / 0.0010000000474974513
    # 0.0010000000474974513  # 深度缩放因子
    
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
    # show_image_and_get_pixel(color)
    print("camera info", camera)
    print("camera scale", camera.scale) 
    
    
    if len(clicked_points) >= 2:
        # Use first point as left_up and second as right_bottom
        x_left_up, y_left_up = clicked_points[0]
        x_right_bottom, y_right_bottom = clicked_points[1]
        
    else:
        # Use defaults but with correct ordering
        x_left_up, y_left_up = 210, 6
        x_right_bottom, y_right_bottom = 576, 419

    # IMPORTANT: Note the order - depth[y, x] not depth[x, y]
    point_z = depth[y_left_up, x_left_up] / camera.scale
    point_x = (x_left_up - camera.cx) * point_z / camera.fx
    point_y = (y_left_up - camera.cy) * point_z / camera.fy
    point_left_up = (point_x, point_y, point_z)

    # Right bottom corner
    point_z = depth[y_right_bottom, x_right_bottom] / camera.scale
    point_x = (x_right_bottom - camera.cx) * point_z / camera.fx
    point_y = (y_right_bottom - camera.cy) * point_z / camera.fy
    point_right_bottom = (point_x, point_y, point_z)


    # 应用掩膜到深度图
    depth[workspace_mask == 0] = 0

    # 应用掩膜到彩色图
    color[workspace_mask == 0] = 0

    print("depth info",depth.shape)
    # 左上角点的深度和坐标
    #print("depth.shape:", depth.shape)

    print("工作区域左上角坐标:", point_left_up)
    print("工作区域右下角坐标:", point_right_bottom)


    # 从深度图像生成有序点云
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)
    
    # 筛选有效的深度点
    valid_mask = depth > 0  # 筛选出深度值大于 0 的点
    cloud_masked = cloud[valid_mask]
    color_masked = color[valid_mask]

    # 随机采样点云
    if len(cloud_masked) >= cfgs.num_point:
        idxs = np.random.choice(len(cloud_masked), cfgs.num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), cfgs.num_point - len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    
    # 获取采样后的点云和颜色信息
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # 创建 Open3D 点云用于可视化（可选）
    o3d_cloud = o3d.geometry.PointCloud()
    o3d_cloud.points = o3d.utility.Vector3dVector(cloud_sampled.astype(np.float32))
    o3d_cloud.colors = o3d.utility.Vector3dVector(color_sampled.astype(np.float32))
    
    # 返回处理后的数据
    ret_dict = {
        'point_clouds': cloud_sampled.astype(np.float32),  # 点云
        'coors': cloud_sampled.astype(np.float32) / cfgs.voxel_size,  # 坐标
        'feats': np.ones_like(cloud_sampled).astype(np.float32),  # 特征
    }
    # 显示 Open3D 点云（可选）
    # o3d.visualization.draw_geometries([o3d_cloud])


    return ret_dict, o3d_cloud, point_left_up, point_right_bottom

def grasp(data_input, cloud_,point_left_up,point_right_bottom):
    moveit_server = MoveItRobotController()

    # 将输入数据进行批处理
    batch_data = minkowski_collate_fn([data_input])
    
    # 初始化抓取网络 GraspNet
    net = GraspNet(seed_feat_dim=cfgs.seed_feat_dim, is_training=False)
    
    # 检查是否有 GPU 可用，并将网络移动到设备上
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    net.to(device)
    
    # 加载检查点文件
    checkpoint = torch.load(cfgs.checkpoint_path)
    net.load_state_dict(checkpoint['model_state_dict'])
    start_epoch = checkpoint['epoch']
    print("-> 加载检查点 %s (epoch: %d)" % (cfgs.checkpoint_path, start_epoch))
    
    # 设置网络为评估模式
    net.eval()
    
    # 将数据移动到设备上
    for key in batch_data:
        if 'list' in key:  # 如果数据是列表，递归地将每个元素移动到设备上
            for i in range(len(batch_data[key])):
                for j in range(len(batch_data[key][i])):
                    batch_data[key][i][j] = batch_data[key][i][j].to(device)
        else:
            batch_data[key] = batch_data[key].to(device)
    
    # 前向传播，预测抓取姿态
    with torch.no_grad():
        end_points = net(batch_data)
        grasp_preds = pred_decode(end_points)  # 输出为1024x17维的抓取预测
    
    # 将抓取预测转换为 GraspGroup 格式
    preds = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(preds)

    # 如果没有有效的抓取姿态，返回 False
    if len(gg) == 0:
        print("未检测到抓取姿态或无有效抓取姿态。")
        return False
    
    # 对抓取姿态进行非极大值抑制（NMS）
    gg.nms()

    def is_targeting_object(grasp_candidate, point_cloud_obj):
        """
        Check if a grasp is targeting an actual object rather than a white/grey table surface.
        It does this by examining the color of points in the point_cloud_obj near the grasp_candidate's center.
        """
        pos = grasp_candidate.translation  # Center of the grasp
        
        # Search radius around the grasp center to find nearby points
        radius = 0.03  # e.g., 3 cm

        # Ensure the point cloud object has points
        if not hasattr(point_cloud_obj, 'points') or len(point_cloud_obj.points) == 0:
            # print(f"Debug: Grasp at {pos}: Point cloud has no points.")
            return False

        cloud_points_array = np.asarray(point_cloud_obj.points)
        
        # Calculate distances from grasp center to all points in the cloud
        distances = np.linalg.norm(cloud_points_array - pos, axis=1)
        nearby_indices = np.where(distances < radius)[0]
        
        # If no points are found near the grasp center, it's unlikely a good grasp for an object
        if len(nearby_indices) == 0:
            # print(f"Debug: Grasp at {pos}: No points found near grasp center (radius: {radius}m).")
            return False

        # Ensure the point cloud object has color information
        if not hasattr(point_cloud_obj, 'colors') or len(point_cloud_obj.colors) == 0:
            print(f"Warning: Grasp at {pos}: Point cloud lacks color information. Cannot reliably distinguish object from table by color.")
            # If color is essential for this check, and it's missing, conservative to return False.
            return False 

        nearby_colors = np.asarray(point_cloud_obj.colors)[nearby_indices]
        
        # Define criteria for a point being "table-like" (e.g., white or light grey)
        table_brightness_threshold = 0.6  # Minimum average intensity for a point to be considered bright (part of table)
        table_achromatic_spread = 0.15    # Maximum difference between R,G,B channels for a point to be achromatic (grey/white)
                                          # Smaller values mean more strictly grey/white.

        num_table_like_points = 0
        for color_val in nearby_colors: # color_val is an [R, G, B] array
            mean_intensity = np.mean(color_val)
            color_spread = np.max(color_val) - np.min(color_val)
            
            # A point is considered "table-like" if it's bright AND not very colorful (achromatic)
            if mean_intensity > table_brightness_threshold and color_spread < table_achromatic_spread:
                num_table_like_points += 1
        
        num_total_nearby_points = len(nearby_indices)
        num_object_like_points = num_total_nearby_points - num_table_like_points
        
        # Define what ratio of "object-like" points is sufficient
        min_object_point_ratio = 0.3 
        
        actual_object_point_ratio = num_object_like_points / num_total_nearby_points

        return actual_object_point_ratio >= min_object_point_ratio

    min_grasp_height_z = 0.48  # Adjust this based on your table height and object characteristics
    
    valid_grasp_list = []
    for g in gg: # Assuming gg is iterable and contains Grasp objects
        if g.translation[2] > min_grasp_height_z:
            if is_targeting_object(g, cloud_): # Pass the grasp candidate and the point cloud
                valid_grasp_list.append(g)


    if len(valid_grasp_list) == 0:
        print("未检测到有效的物体抓取姿态 (结合高度和颜色过滤后)。") # More specific message
        return False

    # Create a new GraspGroup from the filtered grasps
    # Ensure GraspGroup can be initialized from a list of Grasp objects, or adapt as needed
    if isinstance(valid_grasp_list[0].grasp_array, np.ndarray): # Check if Grasp objects have grasp_array
        valid_grasp_arrays = np.vstack([g.grasp_array for g in valid_grasp_list])
        valid_grasps = GraspGroup(valid_grasp_arrays)
    else: # Fallback or alternative initialization if grasp_array is not the primary way
        valid_grasps = GraspGroup(valid_grasp_list) # This depends on GraspGroup constructor

    # Continue with original post-processing
    valid_grasps.sort_by_score() # Assuming GraspGroup has this method
        
    if len(valid_grasps) > 10:
        valid_grasps = valid_grasps[:5] # Assuming this is implemented for GraspGroup


    
    grippers = valid_grasps.to_open3d_geometry_list()  # 将抓取姿态转换为 Open3D 几何对象列表
    grippers[0].paint_uniform_color([0, 1, 0])  # 将得分最高的抓取姿态涂成绿色
    #输出得分最高的抓取姿态的z坐标
    print("抓取姿态的z坐标:",valid_grasps[0].translation[2])

    o3d.visualization.draw_geometries([cloud_, *grippers])  # 可视化点云和抓取姿态
    
    def wait_for_user_confirmation(message, skip_allowed=True):
        """
        Wait for user to press Enter to continue or another key to skip
        
        Args:
            message: Message to display to the user
            skip_allowed: Whether to allow skipping this grasp attempt
            
        Returns:
            bool: True if user pressed Enter, False if they pressed another key to skip
        """
        options = " (press Enter to continue, or any other key to skip)" if skip_allowed else " (press Enter to continue)"
        print("\n" + "-" * 50)
        print(message + options)
        
        # Wait for key press
        user_input = input()
        
        if user_input == "" or not skip_allowed:
            print("Continuing...")
            return True
        else:
            print("Skipping this grasp attempt...")
            return False

    moveit_server.close_gripper(0.0)
    for i in range(len(valid_grasps)):
    # for i in range(2):
        print(f"抓取姿态{i}的得分:", valid_grasps[i].score)
        R_camera2object,t_camera2object = valid_grasps[i].rotation_matrix, valid_grasps[i].translation
        
        t_arm2object, quanterion = camera2base(t_camera2object, R_camera2object)
        print("抓取姿态的quanterion:", quanterion)
        print("抓取姿态的坐标:", t_arm2object)
        # Ask for user confirmation before attempting this grasp
        if not wait_for_user_confirmation(f"Preparing to attempt grasp {i} (score: {valid_grasps[i].score:.4f})"):
            continue  # Skip to next grasp if user doesn't press Enter
            
        # Create TCP target pose
        tcp_pose = Pose()
        tcp_pose.position.x = t_arm2object[0]
        tcp_pose.position.y = t_arm2object[1]
        tcp_pose.position.z = t_arm2object[2]
        
        tcp_pose.orientation.x = quanterion[0]
        tcp_pose.orientation.y = quanterion[1]
        tcp_pose.orientation.z = quanterion[2]
        tcp_pose.orientation.w = quanterion[3]
        
        # Move to pre-grasp position (5cm above)
        import copy
        pre_grasp_pose = copy.deepcopy(tcp_pose)
        # pre_grasp_pose.position.z += 0.05

        # Use the move_to_tcp_pose method
        moveit_server.close_gripper(0.0)
        pre_grasp_success = moveit_server.move_to_pose(pre_grasp_pose)
        # pre_grasp_success = moveit_server.move_to_tcp_pose(pre_grasp_pose)
        if pre_grasp_success:
            # Move to grasp position
            tcp_pose.position.z -= 0.06
            tcp_pose.position.y += 0.05
            # grasp_success = moveit_server.move_to_tcp_pose(tcp_pose)
            grasp_success = moveit_server.move_to_pose(tcp_pose)
            
            if grasp_success:
                # Close gripper
                # sleep some time using rospy
                # Iterate for 10 times to close the gripper
                for _ in range(10):
                    moveit_server.close_gripper(1.8) # 0-1

                # # 移动回 home 位姿
                home_joint_values = moveit_server.manipulator.get_named_target_values("home")
                print("Home joint values:", home_joint_values)
                if not moveit_server.move_to_joint_state(home_joint_values):
                    rospy.logerr("Failed to go home")
                    return False
                return True
    
    else:
        print("未检测到抓取姿态或无有效抓取姿态。")
    return False


def visualize_point_cloud(cloud):
    # Convert cloud to Open3D point cloud format
    if isinstance(cloud, np.ndarray):
        # Assume cloud is a numpy array of shape (N, 3)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(cloud)  # Set the point cloud
    else:
        raise ValueError("The cloud should be a numpy array with shape (N, 3)")
    
    # Visualize the point cloud
    o3d.visualization.draw_geometries([pcd])


if __name__ == '__main__':
    data_dict, cloud, point_left_up, point_right_bottom = data_process()
    success=grasp(data_dict, cloud,point_left_up, point_right_bottom)