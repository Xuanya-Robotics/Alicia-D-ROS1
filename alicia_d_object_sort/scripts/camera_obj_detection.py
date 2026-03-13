import os
import cv2
import yaml
import argparse
import numpy as np
try:
    import rospy
    from geometry_msgs.msg import PoseArray, Pose, PoseStamped
    from std_msgs.msg import Header, String
    from sensor_msgs.msg import Image, CameraInfo
except Exception:
    rospy = None

from object_camera_pose import (
    order_points_clockwise,
    load_intrinsics_from_yaml,
    estimate_plane_from_rectangle,
    pixel_to_plane_point,
)


def _normalize_quaternion(qw: float, qx: float, qy: float, qz: float):
    q = np.array([qw, qx, qy, qz], dtype=np.float64)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return 1.0, 0.0, 0.0, 0.0
    q /= n
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def _quaternion_to_rotation_matrix(qw: float, qx: float, qy: float, qz: float) -> np.ndarray:
    qw, qx, qy, qz = _normalize_quaternion(qw, qx, qy, qz)
    # Standard quaternion to rotation
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz
    R = np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )
    return R


class TransformHelper:
    """Helper for camera->tool0 and camera->base_link transforms.

    - Loads hand-eye `T_tool0_camlink` from calibration yaml file.
    - Supports both USB camera and D405 camera configurations.
    - For D405: handles transform from camera_color_optical_frame to camera_link.
    - Publishes hand-eye TF (tool0 -> camera_link) to complete TF tree.
    - Uses ROS TF to lookup `base_link <- tool0` for camera->base_link.
    """

    # Camera type configurations
    CAMERA_CONFIGS = {
        'usb': {
            'handeye_yaml': 'usb_handeyecalibration_eye_on_hand.yaml',
            'intrinsics_yaml': 'head_camera.yaml',
            'image_topic': None,  # Use cv2.VideoCapture
            'camera_info_topic': None,
            'detection_frame': 'camera_link',  # Frame where detection happens
            'calibration_frame': 'camera_link',  # Frame in hand-eye calibration
        },
        'd405': {
            'handeye_yaml': 'd405_handeyecalibration_eye_on_hand.yaml',
            'intrinsics_yaml': None,  # Get from camera_info topic
            'image_topic': '/camera/color/image_raw',
            'camera_info_topic': '/camera/color/camera_info',
            'detection_frame': 'camera_color_optical_frame',  # Frame where detection happens
            'calibration_frame': 'camera_link',  # Frame in hand-eye calibration
        },
    }

    def __init__(self, enable_ros_tf=True, camera_type='usb'):
        self.T_tool0_camlink = None  # Hand-eye: tool0 -> camera_link
        self.handeye_quat = None  # Store quaternion for TF publishing
        self.handeye_trans = None  # Store translation for TF publishing
        self.ros_ok = False
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_broadcaster = None
        self.shared_detection_mode = 'cubes'
        self.camera_type = camera_type
        self.camera_config = self.CAMERA_CONFIGS.get(camera_type, self.CAMERA_CONFIGS['usb'])
        self._load_handeye_transform()

        if enable_ros_tf:
            try:
                import rospy  # noqa: F401
                import tf2_ros  # noqa: F401
                from std_msgs.msg import String  # noqa: F401

                if not hasattr(self, "rospy"):
                    self.rospy = __import__("rospy")
                    self.tf2_ros = __import__("tf2_ros")

                if not self.rospy.core.is_initialized():
                    self.rospy.init_node("camera_detection_transform", anonymous=True, disable_signals=True)

                self.tf_buffer = self.tf2_ros.Buffer()
                self.tf_listener = self.tf2_ros.TransformListener(self.tf_buffer)
                self.tf_broadcaster = self.tf2_ros.TransformBroadcaster()
                self.ros_ok = True
                self.mode_sub = self.rospy.Subscriber('vision/mode', String, self._on_mode)
                
                # Start publishing hand-eye TF periodically
                if self.T_tool0_camlink is not None:
                    import threading
                    self._tf_publish_thread = threading.Thread(target=self._publish_handeye_tf_loop, daemon=True)
                    self._tf_publish_thread.start()

            except Exception as e:
                print(f"[TransformHelper] TF setup error: {e}")
                try:
                    if rospy is not None:
                        self.rospy = rospy
                        self.ros_ok = True
                        self.mode_sub = self.rospy.Subscriber('vision/mode', String, self._on_mode)
                    else:
                        self.ros_ok = False
                except Exception:
                    self.ros_ok = False

    def _load_handeye_transform(self):
        """Load hand-eye calibration. Stores T_tool0_camlink (tool0 -> camera_link)."""
        handeye_filename = self.camera_config['handeye_yaml']
        yaml_path = os.path.expanduser(f"~/.ros/easy_handeye/{handeye_filename}")
        if not os.path.exists(yaml_path):
            yaml_path = os.path.join(os.path.dirname(__file__), handeye_filename)
        if not os.path.exists(yaml_path):
            print(f"[TransformHelper] Warning: Hand-eye calibration file not found: {handeye_filename}")
            self.T_tool0_camlink = None
            return

        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
            print(f"[TransformHelper] Loaded hand-eye calibration from: {yaml_path}")
        except Exception as e:
            print(f"[TransformHelper] Error loading hand-eye calibration: {e}")
            self.T_tool0_camlink = None
            return

        t = (data or {}).get("transformation", {})
        qw = float(t.get("qw", 1.0))
        qx = float(t.get("qx", 0.0))
        qy = float(t.get("qy", 0.0))
        qz = float(t.get("qz", 0.0))
        tx = float(t.get("x", 0.0))
        ty = float(t.get("y", 0.0))
        tz = float(t.get("z", 0.0))

        # Store for TF publishing
        self.handeye_quat = (qx, qy, qz, qw)  # ROS uses (x, y, z, w)
        self.handeye_trans = (tx, ty, tz)

        # Build transformation matrix: T_tool0_camlink (tool0 -> camera_link)
        R = _quaternion_to_rotation_matrix(qw, qx, qy, qz)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
        self.T_tool0_camlink = T
        print(f"[TransformHelper] Hand-eye transform (tool0 -> camera_link): t=[{tx:.4f}, {ty:.4f}, {tz:.4f}]")

    def _publish_handeye_tf_loop(self):
        """Continuously publish hand-eye TF: tool0 -> camera_link."""
        import geometry_msgs.msg
        rate = self.rospy.Rate(50)  # 50 Hz
        calibration_frame = self.camera_config['calibration_frame']
        
        while not self.rospy.is_shutdown():
            try:
                t = geometry_msgs.msg.TransformStamped()
                t.header.stamp = self.rospy.Time.now()
                t.header.frame_id = "tool0"
                t.child_frame_id = calibration_frame
                t.transform.translation.x = self.handeye_trans[0]
                t.transform.translation.y = self.handeye_trans[1]
                t.transform.translation.z = self.handeye_trans[2]
                t.transform.rotation.x = self.handeye_quat[0]
                t.transform.rotation.y = self.handeye_quat[1]
                t.transform.rotation.z = self.handeye_quat[2]
                t.transform.rotation.w = self.handeye_quat[3]
                self.tf_broadcaster.sendTransform(t)
                rate.sleep()
            except Exception:
                break

    def _on_mode(self, msg):
        m = (msg.data or '').strip().lower()
        if m in ('cubes', 'cans'):
            self.shared_detection_mode = m

    def _get_optical_to_link_transform(self, timeout=0.5):
        """Get transform from camera_color_optical_frame to camera_link via TF."""
        if not self.ros_ok or self.tf_buffer is None:
            return None
        
        detection_frame = self.camera_config['detection_frame']
        calibration_frame = self.camera_config['calibration_frame']
        
        if detection_frame == calibration_frame:
            return np.eye(4, dtype=np.float64)
        
        try:
            tr = self.tf_buffer.lookup_transform(
                calibration_frame, detection_frame,
                self.rospy.Time(0), self.rospy.Duration.from_sec(timeout)
            )
            t = tr.transform.translation
            r = tr.transform.rotation
            R = _quaternion_to_rotation_matrix(r.w, r.x, r.y, r.z)
            T = np.eye(4, dtype=np.float64)
            T[:3, :3] = R
            T[:3, 3] = np.array([t.x, t.y, t.z], dtype=np.float64)
            return T
        except Exception as e:
            print(f"[TransformHelper] Cannot get transform {detection_frame} -> {calibration_frame}: {e}")
            return None

    def optical_to_camlink(self, P_optical):
        """Transform point from detection frame (optical) to calibration frame (camera_link)."""
        T = self._get_optical_to_link_transform()
        if T is None:
            return P_optical  # Fallback: assume same frame
        P_h = np.array([P_optical[0], P_optical[1], P_optical[2], 1.0], dtype=np.float64)
        P_link_h = T @ P_h
        return P_link_h[:3]

    def camera_to_tool(self, Pc):
        """Transform point from detection frame to tool0 frame."""
        if self.T_tool0_camlink is None:
            return None
        
        # First transform from detection frame to camera_link
        Pc_link = self.optical_to_camlink(Pc)
        
        # Then apply inverse of hand-eye: camera_link -> tool0
        # T_tool0_camlink @ P_camlink would give P in a wrong frame
        # We need: P_tool0 = inv(T_tool0_camlink) @ P_camlink ... wait, that's wrong
        # Actually: T_tool0_camlink transforms points FROM camlink TO tool0? No...
        # T_A_B represents pose of B in A, and transforms points FROM B TO A
        # So T_tool0_camlink transforms points from camlink to tool0
        
        Pc_h = np.array([Pc_link[0], Pc_link[1], Pc_link[2], 1.0], dtype=np.float64)
        Pt_h = self.T_tool0_camlink @ Pc_h
        return Pt_h[:3]

    def camera_to_base(self, Pc, lookup_timeout=0.5):
        """Transform point from detection frame to base_link frame."""
        Pt = self.camera_to_tool(Pc)
        if Pt is None:
            return None

        if not self.ros_ok:
            return None

        try:
            tr = self.tf_buffer.lookup_transform("base_link", "tool0", self.rospy.Time(0), self.rospy.Duration.from_sec(lookup_timeout))
            t = tr.transform.translation
            r = tr.transform.rotation
            Rbt = _quaternion_to_rotation_matrix(r.w, r.x, r.y, r.z)
            Tbt = np.eye(4, dtype=np.float64)
            Tbt[:3, :3] = Rbt
            Tbt[0, 3] = t.x
            Tbt[1, 3] = t.y
            Tbt[2, 3] = t.z

            Pt_h = np.array([Pt[0], Pt[1], Pt[2], 1.0], dtype=np.float64)
            Pb_h = Tbt @ Pt_h
            return Pb_h[:3]
        except Exception as e:
            print(f"[TransformHelper] TF lookup base_link<-tool0 failed: {e}")
            return None


def _ros_image_to_cv2(msg):
    """Convert ROS Image message to OpenCV image without cv_bridge C++ backend.
    
    This avoids library conflicts (libffi) that can occur in some Docker environments.
    """
    # Determine the number of channels and dtype based on encoding
    encoding = msg.encoding.lower()
    
    if encoding in ('bgr8', 'rgb8'):
        dtype = np.uint8
        channels = 3
    elif encoding in ('bgra8', 'rgba8'):
        dtype = np.uint8
        channels = 4
    elif encoding == 'mono8':
        dtype = np.uint8
        channels = 1
    elif encoding == 'mono16':
        dtype = np.uint16
        channels = 1
    elif encoding in ('16uc1',):
        dtype = np.uint16
        channels = 1
    elif encoding in ('32fc1',):
        dtype = np.float32
        channels = 1
    else:
        # Fallback: try to parse common encodings
        dtype = np.uint8
        channels = 3
    
    # Convert raw data to numpy array
    img = np.frombuffer(msg.data, dtype=dtype)
    img = img.reshape((msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width))
    
    # Convert RGB to BGR if needed (OpenCV uses BGR)
    if encoding == 'rgb8':
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    elif encoding == 'rgba8':
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA)
    
    return img


class D405ImageReceiver:
    """Helper class to receive images from D405 via ROS topics."""
    
    def __init__(self, image_topic, camera_info_topic):
        self.latest_frame = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.frame_received = False
        
        if rospy is None:
            raise RuntimeError("ROS not available for D405 camera")
        
        # Subscribe to image and camera_info topics
        self.image_sub = rospy.Subscriber(image_topic, Image, self._on_image, queue_size=1)
        self.info_sub = rospy.Subscriber(camera_info_topic, CameraInfo, self._on_camera_info, queue_size=1)
        print(f"[D405] Subscribed to {image_topic} and {camera_info_topic}")
    
    def _on_image(self, msg):
        try:
            # Use manual conversion instead of cv_bridge to avoid library conflicts
            self.latest_frame = _ros_image_to_cv2(msg)
            self.frame_received = True
        except Exception as e:
            print(f"[D405] Error converting image: {e}")
    
    def _on_camera_info(self, msg):
        if self.camera_matrix is None:
            K = np.array(msg.K, dtype=np.float64).reshape(3, 3)
            D = np.array(msg.D, dtype=np.float64)
            self.camera_matrix = K
            self.dist_coeffs = D if len(D) > 0 else np.zeros(5, dtype=np.float64)
            print(f"[D405] Camera intrinsics received: fx={K[0,0]:.2f}, fy={K[1,1]:.2f}, cx={K[0,2]:.2f}, cy={K[1,2]:.2f}")
    
    def get_frame(self):
        return self.latest_frame
    
    def get_intrinsics(self):
        return self.camera_matrix, self.dist_coeffs


def open_camera_with_transforms_dual_mode(camera_index=0, width=None, height=None, enable_ros_tf=True, publish_topics=True, detection_mode="cubes", camera_type='usb'):
    """Open camera feed with dual detection modes: 'cubes' for cube detection, 'cans' for can detection.
    
    - detection_mode: "cubes" or "cans"
    - camera_type: "usb" for USB camera, "d405" for Intel RealSense D405
    - For USB camera: intrinsics loaded from `head_camera.yaml`, hand-eye from `usb_handeyecalibration_eye_on_hand.yaml`
    - For D405: intrinsics from ROS topic, hand-eye from `d405_handeyecalibration_eye_on_hand.yaml`
    - If ROS TF is available, transforms to `base_link` using `base_link <- tool0`
    """
    # Initialize camera based on type
    cap = None
    d405_receiver = None
    
    if camera_type == 'd405':
        print(f"[D405] Initializing Intel RealSense D405 camera...")
        # Initialize ROS node if not already initialized
        if rospy is not None and not rospy.core.is_initialized():
            rospy.init_node("camera_d405_detection", anonymous=True, disable_signals=True)
        
        config = TransformHelper.CAMERA_CONFIGS['d405']
        d405_receiver = D405ImageReceiver(config['image_topic'], config['camera_info_topic'])
        
        # Wait for first frame
        print("[D405] Waiting for camera stream...")
        timeout = 10.0
        start_time = rospy.Time.now() if rospy else None
        while not d405_receiver.frame_received:
            if rospy:
                rospy.sleep(0.1)
                if (rospy.Time.now() - start_time).to_sec() > timeout:
                    print("[D405] Error: Timeout waiting for camera stream")
                    return
        print("[D405] Camera stream received")
    else:
        cap = cv2.VideoCapture(camera_index)
        if width is not None and height is not None:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        if not cap.isOpened():
            print(f"Error: Could not open camera with index {camera_index}.")
            return

    window_name = f"Camera Feed ({camera_type.upper()}) - {detection_mode.upper()} Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    if width is not None and height is not None:
        cv2.resizeWindow(window_name, width, height)

    # Transform helper (hand-eye + optional TF)
    tf_helper = TransformHelper(enable_ros_tf=enable_ros_tf, camera_type=camera_type)
    from tf.transformations import quaternion_from_euler
    goal_orientation = quaternion_from_euler(0, np.pi, 0)
    
    # PoseArray publishers per detection mode
    pubs = {}
    if publish_topics and hasattr(tf_helper, 'rospy'):
        pubs = {
            'cubes/green': tf_helper.rospy.Publisher('vision/cubes/green', PoseArray, queue_size=1),
            'cubes/blue': tf_helper.rospy.Publisher('vision/cubes/blue', PoseArray, queue_size=1),
            # 'cubes/black': tf_helper.rospy.Publisher('vision/cubes/black', PoseArray, queue_size=1),
            'cans/green': tf_helper.rospy.Publisher('vision/cans/green', PoseArray, queue_size=1),
            'cans/blue': tf_helper.rospy.Publisher('vision/cans/blue', PoseArray, queue_size=1),
            'cans/red': tf_helper.rospy.Publisher('vision/cans/red', PoseArray, queue_size=1),
        }

    camera_matrix = None
    dist_coeffs = None
    last_plane = None
    last_px_to_m = None
    px_to_m = None

    while True:
        # Get frame based on camera type
        if camera_type == 'd405':
            frame = d405_receiver.get_frame()
            if frame is None:
                if rospy:
                    rospy.sleep(0.01)
                continue
            ret = True
        else:
            ret, frame = cap.read()
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

        processed_frame = frame.copy()
        if tf_helper.ros_ok and hasattr(tf_helper, 'shared_detection_mode') and tf_helper.shared_detection_mode in ('cubes','cans'):
            detection_mode = tf_helper.shared_detection_mode
        # On-screen mode indicator
        cv2.putText(processed_frame, f"MODE: {detection_mode.upper()} | CAM: {camera_type.upper()}", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)



        paper_corners = None
        if detection_mode == "cubes":
            # Detect A4 paper
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blurred, 75, 200)
            contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) > 0:
                contours = sorted(contours, key=cv2.contourArea, reverse=True)
                for c in contours:
                    perimeter = cv2.arcLength(c, True)
                    approx = cv2.approxPolyDP(c, 0.02 * perimeter, True)
                    if len(approx) == 4:
                        paper_corners = approx
                        cv2.drawContours(processed_frame, [paper_corners], -1, (0, 255, 0), 3)
                        for p in paper_corners:
                            cv2.circle(processed_frame, tuple(p[0]), 10, (0, 0, 255), -1)
                        break

        # Intrinsics once (scaled to stream size)
        if camera_matrix is None:
            H, W = processed_frame.shape[:2]
            if camera_type == 'd405':
                # Get intrinsics from D405 camera_info topic
                K, dist = d405_receiver.get_intrinsics()
                if K is not None:
                    camera_matrix = K
                    dist_coeffs = dist
                else:
                    # Fallback to default intrinsics
                    camera_matrix = np.array([[600.0, 0.0, W / 2.0], [0.0, 600.0, H / 2.0], [0.0, 0.0, 1.0]], dtype=np.float32)
                    dist_coeffs = np.zeros((1, 5), dtype=np.float32)
            else:
                # USB camera: load from yaml file
                yaml_path = os.path.expanduser("~/.ros/camera_info/head_camera.yaml")
                if not os.path.exists(yaml_path):
                    yaml_path = os.path.join(os.path.dirname(__file__), "head_camera.yaml")
                if os.path.exists(yaml_path):
                    K, dist = load_intrinsics_from_yaml(yaml_path, (H, W, 3))
                else:
                    K = np.array([[600.0, 0.0, W / 2.0], [0.0, 600.0, H / 2.0], [0.0, 0.0, 1.0]], dtype=np.float32)
                    dist = np.zeros((1, 5), dtype=np.float32)
                camera_matrix = K
                dist_coeffs = dist

        # Estimate plane from A4 if corners found
        plane = None
        if detection_mode == "cubes":
            if paper_corners is not None and len(paper_corners) == 4:
                pts = paper_corners.reshape(-1, 2).astype(np.float32)
                pts = order_points_clockwise(pts)
                w_px = np.linalg.norm(pts[1] - pts[0])
                h_px = np.linalg.norm(pts[2] - pts[1])
                A4_W, A4_H = (0.297, 0.210) if w_px >= h_px else (0.210, 0.297)
                plane = estimate_plane_from_rectangle(pts, A4_W, A4_H, camera_matrix, dist_coeffs)
                if plane is not None:
                    cv2.polylines(processed_frame, [pts.astype(int)], True, (0, 255, 255), 2)
                    # Pixel-to-meter scale from A4 geometry
                    scale_x = A4_W / max(w_px, 1e-6)
                    scale_y = A4_H / max(h_px, 1e-6)
                    px_to_m = 0.5 * (scale_x + scale_y)
                    last_plane = plane
                    last_px_to_m = px_to_m
        
        else:
            # cans mode: reuse last plane from cubes mode
            if last_plane is not None:
                plane = last_plane
                px_to_m = last_px_to_m

        # Object detection based on mode
        if detection_mode == "cubes":
            processed_frame, detections = detect_cubes_on_frame(processed_frame)
            topic_prefix = 'cubes'
        else:  # detection_mode == "cans"
            processed_frame, detections = detect_cans_on_frame(processed_frame, camera_matrix, dist_coeffs)
            topic_prefix = 'cans'

        # Filter out cube detections that lie outside the detected A4 sheet
        if detection_mode == "cubes" and paper_corners is not None and len(detections) > 0:
            polygon = paper_corners.reshape(-1, 2).astype(np.float32)
            kept = []
            for det in detections:
                cx, cy = det["center_px"]
                inside = cv2.pointPolygonTest(polygon, (float(cx), float(cy)), False)
                if inside >= 0:
                    kept.append(det)
                else:
                    # Visual indicator for filtered-out detections
                    cv2.circle(processed_frame, (int(cx), int(cy)), 6, (0, 0, 255), 2)
            detections = kept


        # Compute 3D positions and transform/publish
        BASE_Z_FIXED = 0.105
        if detection_mode == "cans":
            label_keys = ['green', 'blue', 'red']
        else:
            # label_keys = ['green', 'blue', 'black']
            label_keys = ['green', 'blue', 'read']
        poses_base = {k: [] for k in label_keys}
        poses_tool = {k: [] for k in label_keys}
        if len(detections) > 0:
            # For cubes: need plane intersection; for cans: use PnP-derived camera coords from detections
            n, p0 = (None, None)
            if detection_mode == "cubes" and plane is not None:
                n, p0, _ = plane
            for det in detections:
                cx, cy = det["center_px"]
                label = det.get("label", "obj")
                P_cam = None
                if detection_mode == "cans":
                    P_cam = det.get("center_cam", None)
                elif detection_mode == "cubes" and n is not None and p0 is not None:
                    P_cam = pixel_to_plane_point(cx, cy, camera_matrix, n, p0)

                if P_cam is None:
                    continue

                P_base = tf_helper.camera_to_base(P_cam)
                if P_base is not None:
                    if detection_mode == "cans":
                        P_base = np.array([P_base[0], P_base[1], BASE_Z_FIXED], dtype=np.float64)
                        # Overlay base frame pose for cans
                        x, y = int(cx), int(cy)
                        text_base = f"{label} can base: {P_base[0]:.3f},{P_base[1]:.3f},{P_base[2]:.3f} m"
                        cv2.putText(processed_frame, text_base, (x + 6, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
                    else:
                        # For cubes, still overlay camera coords
                        obj_type = "cube"
                        x, y = int(cx), int(cy)
                        # text_cam = f"{label} {obj_type} C: {P_cam[0]:.3f},{P_cam[1]:.3f},{P_cam[2]:.3f} m"
                        text_base = f"{label} cube base: {P_base[0]:.3f}, {P_base[1]:.3f}, {P_base[2]:.3f} m"
                        cv2.putText(processed_frame, text_base, (x + 6, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)
                    if label in poses_base:
                        poses_base[label].append(P_base)
                else:
                    P_tool = tf_helper.camera_to_tool(P_cam)
                    if P_tool is not None and label in poses_tool:
                        # Overlay tool0 pose if base not available (cans still fixed-Z when ultimately used)
                        if detection_mode == "cans":
                            x, y = int(cx), int(cy)
                            text_tool = f"{label} can tool0: {P_tool[0]:.3f},{P_tool[1]:.3f},{P_tool[2]:.3f} m"
                            cv2.putText(processed_frame, text_tool, (x + 6, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 255, 200), 2)
                        poses_tool[label].append(P_tool)
                    else:
                        # Fallback overlay: camera coords
                        obj_type = "can" if detection_mode == "cans" else "cube"
                        x, y = int(cx), int(cy)
                        text_cam = f"{label} {obj_type} C: {P_cam[0]:.3f},{P_cam[1]:.3f},{P_cam[2]:.3f} m"
                        cv2.putText(processed_frame, text_cam, (x + 6, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

        # Publish PoseArrays
        if publish_topics and hasattr(tf_helper, 'rospy') and (len(detections) > 0):
            now = tf_helper.rospy.Time.now()
            for label in poses_base.keys():
                arr = poses_base[label] if len(poses_base[label]) > 0 else poses_tool[label]
                if len(arr) == 0:
                    continue
                frame_id = 'base_link' if len(poses_base[label]) > 0 else 'tool0'
                pa = PoseArray()
                pa.header = Header(stamp=now, frame_id=frame_id)
                for p in arr:
                    pose = Pose()
                    pose.position.x = float(p[0]); pose.position.y = float(p[1]); pose.position.z = float(p[2])
                    pose.orientation.w = 1.0
                    pa.poses.append(pose)
                topic_key = f'{topic_prefix}/{label}'
                if topic_key in pubs:
                    pubs[topic_key].publish(pa)

        cv2.imshow(window_name, processed_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            cv2.imwrite(f"capture_{camera_type}_{detection_mode}_detection.jpg", processed_frame)
            print(f"Saved image with {detection_mode} detection overlay ({camera_type} camera).")
            break

    # Cleanup
    if cap is not None:
        cap.release()
    cv2.destroyAllWindows()


def detect_cans_on_frame(frame, camera_matrix, dist_coeffs):
    """Enhanced can detection for drop zone. Detects green, blue, and gray cans.
    
    Returns: (image_with_overlays, detections)
    detection: {"label": str, "center_px": (cx, cy), "box": np.ndarray, "center_cam": np.ndarray}
    """
    image = frame.copy()
    brightness = np.mean(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    
    # Adjust parameters based on lighting conditions
    if brightness < 76.5:
        gamma = 2.0
        clahe_limit = 4.0
    elif brightness < 127.5:
        gamma = 1.7
        clahe_limit = 3.5
    else:
        gamma = 1.3
        clahe_limit = 2.5
        
    # Gamma correction for low light
    inv_gamma = 1.0 / gamma
    gamma_table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    image = cv2.LUT(image, gamma_table)
    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_image)
    
    # CLAHE enhancement
    clahe = cv2.createCLAHE(clipLimit=clahe_limit, tileGridSize=(8, 8))
    v = clahe.apply(v)
    s = clahe.apply(s)
    hsv_image = cv2.merge([h, s, v])
    
    # Dynamic color range thresholds based on brightness (more conservative for cans)
    min_sat = max(80, 140 - int((127.5 - brightness) * 0.8)) if brightness < 127.5 else 110
    min_val = max(70, 130 - int((127.5 - brightness) * 0.5)) if brightness < 127.5 else 110
    
    # Color ranges for cans (replace gray with red); use tighter blue range to reduce noise
    blue_h_lo, blue_h_hi = 100, 130
    blue_min_sat = max(min_sat, 140)
    blue_min_val = max(min_val, 100)
    lower_blue = np.array([blue_h_lo, blue_min_sat, blue_min_val], dtype="uint8")
    upper_blue = np.array([blue_h_hi, 255, 255], dtype="uint8")
    # green as before
    lower_green = np.array([35, min_sat, min_val], dtype="uint8")
    upper_green = np.array([85, 255, 255], dtype="uint8")
    # red wrap-around
    lower_red1 = np.array([0, min_sat, min_val], dtype="uint8")
    upper_red1 = np.array([10, 255, 255], dtype="uint8")
    lower_red2 = np.array([170, min_sat, min_val], dtype="uint8")
    upper_red2 = np.array([180, 255, 255], dtype="uint8")
    
    # Physical size filter (pixel-based fallback)
    min_area = max(300, int(500 * (brightness / 255.0)))
    
    # Build masks including merged red
    masks = {
        "blue": cv2.inRange(hsv_image, lower_blue, upper_blue),
        "green": cv2.inRange(hsv_image, lower_green, upper_green),
        "red": (cv2.inRange(hsv_image, lower_red1, upper_red1) | cv2.inRange(hsv_image, lower_red2, upper_red2)),
    }

    detections = []
    for color_name, mask in masks.items():
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.GaussianBlur(mask, (3, 3), 0)
        
        # Morphological operations for can shapes (more elongated). Use stronger smoothing for blue
        if color_name == "blue":
            kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (14, 14))
        else:
            kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (6, 6))
            kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        iterations = 2 if brightness < 127.5 else 1
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=iterations)
        
        if brightness < 76.5:
            kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
            mask = cv2.dilate(mask, kernel_dilate, iterations=1)
            
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area_px = cv2.contourArea(contour)
            min_area_color = max(min_area, 1000) if color_name == "blue" else min_area
            if area_px < min_area_color:
                continue
            hull = cv2.convexHull(contour)
            rect = cv2.minAreaRect(hull)
            (cx, cy), (w, h), _ = rect
            # Pixel threshold
            if min(w, h) < 20:
                continue
            # Cans/drop bins appear as elongated rectangles; filter by aspect and extent
            aspect = max(w, h) / max(min(w, h), 1e-6)
            aspect_lo = 1.25 if color_name == "blue" else 1.15
            if not aspect_lo <= aspect <= 3.0:
                continue
            rect_area = max(w * h, 1e-6)
            extent = float(area_px) / rect_area
            extent_lo = 0.50 if color_name == "blue" else 0.40
            if extent < extent_lo:  # suppress thin/noisy blobs
                continue
            box = cv2.boxPoints(rect)
            # Order corners TL, TR, BR, BL
            ordered = order_points_clockwise(box.astype(np.float32))
            # Known can dimensions (meters)
            CAN_W_M = 0.085
            CAN_H_M = 0.095
            # Estimate pose of the rectangle using PnP to get camera-frame center
            center_cam = None
            try:
                plane = estimate_plane_from_rectangle(ordered, CAN_W_M, CAN_H_M, camera_matrix, dist_coeffs)
                if plane is not None:
                    _, p0, (rvec, tvec) = plane
                    R, _ = cv2.Rodrigues(rvec)
                    center_obj = np.array([CAN_W_M * 0.5, CAN_H_M * 0.5, 0.0], dtype=np.float32)
                    center_cam = (R @ center_obj.reshape(3, 1) + tvec).reshape(3)
            except Exception:
                center_cam = None
            box = box.astype(int)
            cv2.polylines(image, [box], True, (255, 165, 0), 2)  # Orange for cans
            cv2.circle(image, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            cv2.putText(image, f"{color_name}_can", (box[0][0], box[0][1] - 6), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)
            
            detections.append({
                "label": color_name,
                "center_px": (float(cx), float(cy)),
                "box": box,
                "center_cam": center_cam,
            })
            
    return image, detections

def detect_cubes_on_frame(frame):
    """Enhanced lightweight live cube detection optimized for low-light conditions.

    Returns: (image_with_overlays, detections)
    detection: {"label": str, "center_px": (cx, cy), "box": np.ndarray}
    """
    image = frame.copy()
    brightness = np.mean(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))

    if brightness < 76.5:
        gamma = 2.0
        clahe_limit = 4.0
    elif brightness < 127.5:
        gamma = 1.7
        clahe_limit = 3.5
    else:
        gamma = 1.3
        clahe_limit = 2.5

    inv_gamma = 1.0 / gamma
    gamma_table = np.array([((i / 255.0) ** inv_gamma) * 255 for i in np.arange(0, 256)]).astype("uint8")
    image = cv2.LUT(image, gamma_table)

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv_image)

    clahe = cv2.createCLAHE(clipLimit=clahe_limit, tileGridSize=(8, 8))
    v = clahe.apply(v)
    s = clahe.apply(s)
    hsv_image = cv2.merge([h, s, v])

    min_sat = max(20, 100 - int((127.5 - brightness) * 0.6)) if brightness < 127.5 else 80
    min_val = max(20, 100 - int((127.5 - brightness) * 0.6)) if brightness < 127.5 else 80

    color_ranges = {
        "blue": ([90, min_sat, min_val], [130, 255, 255]),
        "green": ([35, min_sat, min_val], [85, 255, 255]),
        # "black": ([0, 0, 0], [180, 255, 40]),
    }
    min_area = max(150, int(250 * (brightness / 255.0)))

    detections = []
    for color_name, (lower_bound, upper_bound) in color_ranges.items():
        lower = np.array(lower_bound, dtype="uint8")
        upper = np.array(upper_bound, dtype="uint8")

        mask = cv2.inRange(hsv_image, lower, upper)
        mask = cv2.medianBlur(mask, 5)
        mask = cv2.GaussianBlur(mask, (3, 3), 0)

        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (4, 4))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        iterations = 2 if brightness < 127.5 else 1
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=iterations)

        if brightness < 76.5:
            kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.dilate(mask, kernel_dilate, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if cv2.contourArea(contour) < min_area:
                continue
            hull = cv2.convexHull(contour)
            rect = cv2.minAreaRect(hull)
            (cx, cy), (w, h), _ = rect
            if min(w, h) < 12:
                continue
            aspect = (w / h) if h > 1e-6 else 0
            if not 0.7 <= aspect <= 1.4:
                continue
            box = cv2.boxPoints(rect).astype(int)
            cv2.polylines(image, [box], True, (0, 255, 0), 2)
            cv2.circle(image, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(image, color_name, (box[0][0], box[0][1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            # print(f"color_name: {color_name}, cx: {cx}, cy: {cy}, w: {w}, h: {h}")
            detections.append({
                "label": color_name,
                "center_px": (float(cx), float(cy)),
                "box": box,
            })

    return image, detections


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Camera object detection with hand-eye calibration')
    parser.add_argument('--camera', '-c', type=str, default='usb', choices=['usb', 'd405'],
                        help='Camera type: usb (default) or d405 (Intel RealSense D405)')
    parser.add_argument('--width', '-W', type=int, default=640,
                        help='Frame width (default: 640)')
    parser.add_argument('--height', '-H', type=int, default=480,
                        help='Frame height (default: 480)')
    parser.add_argument('--index', '-i', type=int, default=0,
                        help='USB camera index (default: 0, only for USB camera)')
    parser.add_argument('--mode', '-m', type=str, default='cubes', choices=['cubes', 'cans'],
                        help='Detection mode: cubes (default) or cans')
    args = parser.parse_args()
    
    print(f"Starting camera detection with:")
    print(f"  Camera type: {args.camera}")
    print(f"  Resolution: {args.width}x{args.height}")
    print(f"  Detection mode: {args.mode}")
    if args.camera == 'usb':
        print(f"  Camera index: {args.index}")
    
    open_camera_with_transforms_dual_mode(
        camera_index=args.index,
        width=args.width,
        height=args.height,
        enable_ros_tf=True,
        camera_type=args.camera,
        detection_mode=args.mode
    )

