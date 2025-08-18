#!/usr/bin/env python3
"""
ChArUco Tracker Node for Eye-on-Hand Calibration

This node detects ChArUco boards and publishes their pose as a TF frame,
making it compatible with easy_handeye calibration system.

It also publishes a debug image with the pose axis drawn on it to the
/charuco/result topic.
"""

import rospy
import cv2
import cv2.aruco as aruco
import numpy as np
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge, CvBridgeError
import message_filters

class CharucoTracker:
    def __init__(self):
        rospy.init_node('charuco_tracker', anonymous=True)
        
        # Parameters
        board_size_param = rospy.get_param('~board_size', [11, 8])  # Width x Height
        # Handle both string and list formats for board_size
        if isinstance(board_size_param, str):
            # Parse string format like "[11, 8]"
            import ast
            try:
                self.board_size = ast.literal_eval(board_size_param)
            except:
                rospy.logwarn(f"Failed to parse board_size string: {board_size_param}, using default [11, 8]")
                self.board_size = [11, 8]
        else:
            self.board_size = board_size_param
            
        # Ensure board_size is a tuple of exactly 2 integers
        if not isinstance(self.board_size, (list, tuple)) or len(self.board_size) != 2:
            rospy.logwarn(f"Invalid board_size format: {self.board_size}, using default [11, 8]")
            self.board_size = [11, 8]
        
        self.board_size = tuple(map(int, self.board_size))  # Convert to tuple of ints
        
        self.square_length = rospy.get_param('~square_length', 0.015)  # 15mm
        self.marker_length = rospy.get_param('~marker_length', 0.011)  # 11mm
        self.dictionary_id = rospy.get_param('~dictionary_id', 'DICT_4X4_100')
        self.camera_frame = rospy.get_param('~camera_frame', 'camera_link')
        self.board_frame = rospy.get_param('~board_frame', 'charuco_board')
        
        # Camera calibration parameters (should be loaded from camera_info)
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # ChArUco board setup
        self.dictionary = aruco.getPredefinedDictionary(getattr(aruco, self.dictionary_id))
        self.board = aruco.CharucoBoard(self.board_size, self.square_length, self.marker_length, self.dictionary)
        
        # Set legacy pattern for OpenCV 4.x compatibility
        if hasattr(self.board, 'setLegacyPattern'):
            self.board.setLegacyPattern(True)
            rospy.loginfo("Legacy pattern enabled for OpenCV 4.x compatibility")
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        
        # CV bridge
        self.bridge = CvBridge()
        
        # Publisher for the result image
        self.result_pub = rospy.Publisher('/charuco/result', Image, queue_size=1)
        
        # Subscribers
        self.image_sub = message_filters.Subscriber('/usb_cam/image_raw', Image)
        self.camera_info_sub = message_filters.Subscriber('/usb_cam/camera_info', CameraInfo)
        
        # Synchronize image and camera info
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.camera_info_sub], 10)
        self.ts.registerCallback(self.callback)
        
        rospy.loginfo("ChArUco tracker initialized")
        rospy.loginfo(f"Board size: {self.board_size[0]}x{self.board_size[1]}")
        rospy.loginfo(f"Square length: {self.square_length*1000:.1f}mm")
        rospy.loginfo(f"Marker length: {self.marker_length*1000:.1f}mm")
    
    def callback(self, image_msg, camera_info_msg):
        """Process synchronized image and camera info messages."""
        
        # Update camera parameters if needed
        if self.camera_matrix is None:
            self.camera_matrix = np.array(camera_info_msg.K).reshape(3, 3)
            self.dist_coeffs = np.array(camera_info_msg.D)
            rospy.loginfo("Camera parameters loaded")
        
        # Convert ROS image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except Exception as e:
            rospy.logwarn(f"Failed to convert image: {e}")
            return
        
        # Detect ChArUco board
        charuco_corners, charuco_ids = self.detect_charuco_board(cv_image)
        
        if charuco_corners is not None and charuco_ids is not None:
            # Estimate pose
            pose, rvec, tvec = self.estimate_pose(charuco_corners, charuco_ids)
            if pose is not None:
                # Publish TF transform
                self.publish_tf(pose, image_msg.header.stamp)
                
                # Draw the pose axis on the image for visualization
                try:
                    # Check if axes will be within image bounds before drawing
                    if self._can_draw_axes_safely(rvec, tvec, cv_image.shape):
                        # Use a fixed axis length that's appropriate for visualization
                        # 0.03 meters (3cm) is a good compromise between visibility and staying in frame
                        axis_length = 0.03
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, axis_length)
                    else:
                        # If axes would go out of frame, just draw a small marker at the origin
                        origin_2d, _ = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec, tvec, 
                                                       self.camera_matrix, self.dist_coeffs)
                        origin_2d = tuple(map(int, origin_2d[0, 0]))
                        cv2.circle(cv_image, origin_2d, 5, (0, 255, 0), -1)  # Green circle at origin
                except AttributeError:
                    # Fallback to the older aruco.drawAxis for compatibility
                    if self._can_draw_axes_safely(rvec, tvec, cv_image.shape):
                        aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03)
                    else:
                        # Draw origin marker instead
                        origin_2d, _ = cv2.projectPoints(np.array([[0.0, 0.0, 0.0]]), rvec, tvec, 
                                                       self.camera_matrix, self.dist_coeffs)
                        origin_2d = tuple(map(int, origin_2d[0, 0]))
                        cv2.circle(cv_image, origin_2d, 5, (0, 255, 0), -1)

        # Publish the final image with annotations (if any)
        try:
            result_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            result_msg.header = image_msg.header
            self.result_pub.publish(result_msg)
        except CvBridgeError as e:
            rospy.logerr(f"Failed to publish result image: {e}")

    def detect_charuco_board(self, image):
        """Detect ChArUco board in the image."""
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers first
            detector_params = aruco.DetectorParameters()
            detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE  # Important for ChArUco
            
            detector = aruco.ArucoDetector(self.dictionary, detector_params)
            marker_corners, marker_ids, _ = detector.detectMarkers(gray)
            
            if marker_ids is None or len(marker_ids) == 0:
                return None, None
            
            # Detect ChArUco corners
            charuco_detector = cv2.aruco.CharucoDetector(self.board, detectorParams=detector_params)
            # --- SOLUTION CHANGE 2: Use the grayscale image for detection for consistency ---
            charuco_corners, charuco_ids, _, _ = charuco_detector.detectBoard(gray)
            
            return charuco_corners, charuco_ids
            
        except Exception as e:
            rospy.logwarn(f"ChArUco detection failed: {e}")
            return None, None
    
    def estimate_pose(self, charuco_corners, charuco_ids):
        """Estimate pose of the ChArUco board."""
        try:
            # Estimate pose using ChArUco corners
            retval, rvec, tvec = aruco.estimatePoseCharucoBoard(
                charuco_corners, charuco_ids, self.board, 
                self.camera_matrix, self.dist_coeffs, 
                None, None, useExtrinsicGuess=False
            )
            
            if retval:
                # Convert rotation vector to rotation matrix
                R, _ = cv2.Rodrigues(rvec)
                
                # Create transformation matrix
                T = np.eye(4)
                T[:3, :3] = R
                T[:3, 3] = tvec.flatten()
                
                return T, rvec, tvec
            else:
                return None, None, None
                
        except Exception as e:
            rospy.logwarn(f"Pose estimation failed: {e}")
            return None, None, None

    def publish_tf(self, pose_matrix, timestamp):
        """Publish the board pose as a TF transform."""
        try:
            # Extract rotation and translation
            R = pose_matrix[:3, :3]
            t = pose_matrix[:3, 3]
            
            # Use scipy for more reliable rotation matrix to quaternion conversion
            from scipy.spatial.transform import Rotation
            
            # Apply coordinate system transformation
            T_cv_to_ros = np.array([
                [0, 0, 1],    # OpenCV Z -> ROS X
                [-1, 0, 0],   # OpenCV -X -> ROS Y  
                [0, -1, 0]    # OpenCV -Y -> ROS Z
            ])
            
            R_ros = T_cv_to_ros @ R
            t_ros = T_cv_to_ros @ t
            
            # Convert rotation matrix to quaternion
            rotation = Rotation.from_matrix(R_ros)
            quat = rotation.as_quat()  # Returns [x, y, z, w]
            
            # Create transform message
            transform = TransformStamped()
            transform.header.stamp = timestamp
            transform.header.frame_id = self.camera_frame
            transform.child_frame_id = self.board_frame
            
            transform.transform.translation.x = t_ros[0]
            transform.transform.translation.y = t_ros[1]
            transform.transform.translation.z = t_ros[2]
            
            transform.transform.rotation.x = quat[0]
            transform.transform.rotation.y = quat[1]
            transform.transform.rotation.z = quat[2]
            transform.transform.rotation.w = quat[3]
            
            self.tf_broadcaster.sendTransform(transform)
            
        except ImportError:
            rospy.logwarn_once("scipy not available, falling back to manual quaternion conversion")
            self._publish_tf_manual(pose_matrix, timestamp)
        except Exception as e:
            rospy.logwarn(f"Failed to publish TF: {e}")
    
    def _publish_tf_manual(self, pose_matrix, timestamp):
        """Fallback manual TF publishing without scipy."""
        try:
            # Extract rotation and translation
            R = pose_matrix[:3, :3]
            t = pose_matrix[:3, 3]
            
            # Apply coordinate system transformation
            T_cv_to_ros = np.array([
                [0, 0, 1],    # OpenCV Z -> ROS X
                [-1, 0, 0],   # OpenCV -X -> ROS Y  
                [0, -1, 0]    # OpenCV -Y -> ROS Z
            ])
            
            R_ros = T_cv_to_ros @ R
            t_ros = T_cv_to_ros @ t
            
            # Manual rotation matrix to quaternion conversion
            trace = np.trace(R_ros)
            if trace > 0:
                S = np.sqrt(trace + 1.0) * 2
                w = 0.25 * S
                x = (R_ros[2, 1] - R_ros[1, 2]) / S
                y = (R_ros[0, 2] - R_ros[2, 0]) / S
                z = (R_ros[1, 0] - R_ros[0, 1]) / S
            else:
                if R_ros[0, 0] > R_ros[1, 1] and R_ros[0, 0] > R_ros[2, 2]:
                    S = np.sqrt(1.0 + R_ros[0, 0] - R_ros[1, 1] - R_ros[2, 2]) * 2
                    w = (R_ros[2, 1] - R_ros[1, 2]) / S
                    x = 0.25 * S
                    y = (R_ros[0, 1] + R_ros[1, 0]) / S
                    z = (R_ros[0, 2] + R_ros[2, 0]) / S
                elif R_ros[1, 1] > R_ros[2, 2]:
                    S = np.sqrt(1.0 + R_ros[1, 1] - R_ros[0, 0] - R_ros[2, 2]) * 2
                    w = (R_ros[0, 2] - R_ros[2, 0]) / S
                    x = (R_ros[0, 1] + R_ros[1, 0]) / S
                    y = 0.25 * S
                    z = (R_ros[1, 2] + R_ros[2, 1]) / S
                else:
                    S = np.sqrt(1.0 + R_ros[2, 2] - R_ros[0, 0] - R_ros[1, 1]) * 2
                    w = (R_ros[1, 0] - R_ros[0, 1]) / S
                    x = (R_ros[0, 2] + R_ros[2, 0]) / S
                    y = (R_ros[1, 2] + R_ros[2, 1]) / S
                    z = 0.25 * S
            
            # Create transform message
            transform = TransformStamped()
            transform.header.stamp = timestamp
            transform.header.frame_id = self.camera_frame
            transform.child_frame_id = self.board_frame
            
            transform.transform.translation.x = t_ros[0]
            transform.transform.translation.y = t_ros[1]
            transform.transform.translation.z = t_ros[2]
            
            transform.transform.rotation.x = x
            transform.transform.rotation.y = y
            transform.transform.rotation.z = z
            transform.transform.rotation.w = w
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            rospy.logwarn(f"Failed to publish TF manually: {e}")
    
    def _can_draw_axes_safely(self, rvec, tvec, image_shape):
        """Check if drawing axes will keep endpoints within image bounds."""
        try:
            # Test with a reasonable axis length (3cm)
            axis_length = 0.03
            height, width = image_shape[:2]
            
            # Define axis endpoints in 3D (origin + X, Y, Z axes)
            axis_points = np.array([
                [0.0, 0.0, 0.0],           # Origin
                [axis_length, 0.0, 0.0],   # X-axis end
                [0.0, axis_length, 0.0],   # Y-axis end
                [0.0, 0.0, axis_length]    # Z-axis end
            ], dtype=np.float32)
            
            # Project 3D points to 2D image coordinates
            projected_points, _ = cv2.projectPoints(axis_points, rvec, tvec, 
                                                  self.camera_matrix, self.dist_coeffs)
            projected_points = projected_points.reshape(-1, 2)
            
            # Check if all projected points are within image bounds
            # Add a small margin to avoid drawing very close to edges
            margin = 10
            for point in projected_points:
                x, y = point
                if (x < margin or x > width - margin or 
                    y < margin or y > height - margin):
                    return False
            
            return True
            
        except Exception as e:
            rospy.logwarn(f"Error checking axis bounds: {e}")
            return False  # If we can't check, don't draw axes

if __name__ == '__main__':
    try:
        tracker = CharucoTracker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass