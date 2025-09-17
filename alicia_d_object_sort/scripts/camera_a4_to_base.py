import os
import cv2
import yaml
import numpy as np
try:
    import rospy
    from geometry_msgs.msg import PoseArray, Pose, PoseStamped
    from std_msgs.msg import Header, String
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

    - Loads hand-eye `T_cam_tool0` from `usb_handeyecalibration_eye_on_hand.yaml`.
    - Optionally uses ROS TF to lookup `base_link <- tool0` for camera->base_link.
    """

    def __init__(self, enable_ros_tf=True):
        self.T_cam_tool0 = None
        self.ros_ok = False
        self.tf_buffer = None
        self.tf_listener = None
        self.tf_broadcaster = None
        self.mode_sub = None
        self.shared_detection_mode = 'cubes'

        self._load_handeye_transform()

        if enable_ros_tf:
            try:
                import rospy  # noqa: F401
                import tf2_ros  # noqa: F401

                if not hasattr(self, "rospy"):
                    # Bind for internal use without making them hard deps for import
                    self.rospy = __import__("rospy")
                    self.tf2_ros = __import__("tf2_ros")

                if not self.rospy.core.is_initialized():
                    # Anonymous node allows reuse if already running
                    self.rospy.init_node("camera_a4_transform", anonymous=True, disable_signals=True)

                self.tf_buffer = self.tf2_ros.Buffer()
                self.tf_listener = self.tf2_ros.TransformListener(self.tf_buffer)
                self.tf_broadcaster = self.tf2_ros.TransformBroadcaster()
                self.ros_ok = True
                # Optional detection mode subscriber
                try:
                    self.mode_sub = self.rospy.Subscriber('vision/mode', String, self._on_mode)
                except Exception:
                    self.mode_sub = None
            except Exception:
                # ROS/TF not available; camera->base will be skipped
                self.ros_ok = False

    def _load_handeye_transform(self):
        yaml_path = os.path.join(os.path.dirname(__file__), "usb_handeyecalibration_eye_on_hand.yaml")
        if not os.path.exists(yaml_path):
            self.T_cam_tool0 = None
            return

        try:
            with open(yaml_path, "r") as f:
                data = yaml.safe_load(f)
        except Exception:
            self.T_cam_tool0 = None
            return

        t = (data or {}).get("transformation", {})
        qw = float(t.get("qw", 1.0))
        qx = float(t.get("qx", 0.0))
        qy = float(t.get("qy", 0.0))
        qz = float(t.get("qz", 0.0))
        tx = float(t.get("x", 0.0))
        ty = float(t.get("y", 0.0))
        tz = float(t.get("z", 0.0))

        R = _quaternion_to_rotation_matrix(qw, qx, qy, qz)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
        self.T_cam_tool0 = T

    # ROS callback to switch detection mode externally
    def _on_mode(self, msg):
        m = (msg.data or '').strip().lower()
        if m in ('cubes', 'cans'):
            self.shared_detection_mode = m

    def camera_to_tool(self, Pc):
        if self.T_cam_tool0 is None:
            return None
        Pc_h = np.array([Pc[0], Pc[1], Pc[2], 1.0], dtype=np.float64)
        Pt_h = self.T_cam_tool0 @ Pc_h
        return Pt_h[:3]

    def camera_to_base(self, Pc, lookup_timeout=0.1):
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
        except Exception:
            return None


def open_camera_with_transforms_dual_mode(camera_index=0, width=None, height=None, enable_ros_tf=True, publish_topics=True, detection_mode="cubes"):
    """Open camera feed with dual detection modes: 'cubes' for cube detection, 'cans' for can detection.
    
    - detection_mode: "cubes" or "cans"
    - Camera intrinsics loaded from `head_camera.yaml` in this directory, scaled to stream size
    - Hand-eye calibration loaded from `usb_handeyecalibration_eye_on_hand.yaml` in this directory  
    - If ROS TF is available, transforms to `base_link` using `base_link <- tool0`
    """
    cap = cv2.VideoCapture(camera_index)
    if width is not None and height is not None:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    if not cap.isOpened():
        print(f"Error: Could not open camera with index {camera_index}.")
        return

    window_name = "Camera Feed - Dual Detection"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    if width is not None and height is not None:
        cv2.resizeWindow(window_name, width, height)

    # Transform helper (hand-eye + optional TF)
    tf_helper = TransformHelper(enable_ros_tf=enable_ros_tf)
    from tf.transformations import quaternion_from_euler
    goal_orientation = quaternion_from_euler(0, np.pi, 0)
    
    # PoseArray publishers: create ALL upfront so switching modes publishes correctly
    pubs = {}
    if publish_topics and tf_helper.ros_ok:
        pubs = {
            'cubes/green': tf_helper.rospy.Publisher('vision/cubes/green', PoseArray, queue_size=1),
            'cubes/blue': tf_helper.rospy.Publisher('vision/cubes/blue', PoseArray, queue_size=1),
            'cubes/black': tf_helper.rospy.Publisher('vision/cubes/black', PoseArray, queue_size=1),
            'cans/green': tf_helper.rospy.Publisher('vision/cans/green', PoseArray, queue_size=1),
            'cans/blue': tf_helper.rospy.Publisher('vision/cans/blue', PoseArray, queue_size=1),
            'cans/red': tf_helper.rospy.Publisher('vision/cans/red', PoseArray, queue_size=1),
        }

    camera_matrix = None
    dist_coeffs = None
    # Persist A4 plane and scale from cubes mode for use in cans mode
    last_plane = None  # (n, p0, (rvec, tvec))
    last_px_to_m = None

    FIXED_Z_BASE_M = 0.20  # Publish constant height in base frame

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Can't receive frame (stream end?). Exiting ...")
            break

        processed_frame = frame.copy()
        # Enforce external mode via topic (published by cube_sorting)
        if tf_helper.ros_ok and hasattr(tf_helper, 'shared_detection_mode') and tf_helper.shared_detection_mode in ('cubes','cans'):
            detection_mode = tf_helper.shared_detection_mode
        # On-screen mode indicator
        cv2.putText(processed_frame, f"MODE: {detection_mode.upper()} (via topic 'vision/mode')", (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        # Detect A4 paper ONLY in cubes mode; persist plane for cans mode
        paper_corners = None
        if detection_mode == "cubes":
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
            yaml_path = os.path.join(os.path.dirname(__file__), "head_camera.yaml")
            H, W = processed_frame.shape[:2]
            if os.path.exists(yaml_path):
                K, dist = load_intrinsics_from_yaml(yaml_path, (H, W, 3))
            else:
                K = np.array([[600.0, 0.0, W / 2.0], [0.0, 600.0, H / 2.0], [0.0, 0.0, 1.0]], dtype=np.float32)
                dist = np.zeros((1, 5), dtype=np.float32)
            camera_matrix = K
            dist_coeffs = dist

        # Estimate plane from A4 if corners found (cubes mode), or reuse last known plane (cans mode)
        plane = None
        px_to_m = None  # average pixel-to-meter scale on the A4 plane (approx)
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
                    # Approx pixel-to-meter scale using A4 known dimensions
                    scale_x = A4_W / max(w_px, 1e-6)
                    scale_y = A4_H / max(h_px, 1e-6)
                    px_to_m = float(0.5 * (scale_x + scale_y))
                    # Persist for cans mode
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
            processed_frame, detections = detect_cans_on_frame(processed_frame, px_to_m=px_to_m)
            topic_prefix = 'cans'

        # If plane available, compute 3D positions and transform
        if plane is not None and len(detections) > 0:
            n, p0, _ = plane
            poses_base = {'green': [], 'blue': [], 'black': [], 'red': []}
            for det in detections:
                cx, cy = det["center_px"]
                P_cam = pixel_to_plane_point(cx, cy, camera_matrix, n, p0)
                label = det.get("label", "obj")
                
                # For cans, add height offset (10cm above A4 plane)
                if detection_mode == "cans" and P_cam is not None:
                    can_height = det.get("height", 0.10)
                    # Adjust along plane normal (up from plane)
                    P_cam = P_cam - can_height * n.flatten()
                    # P_cam = P_cam + can_height * n.flatten()

                # Overlay and print camera coords
                if P_cam is not None:
                    obj_type = "can" if detection_mode == "cans" else "cube"
                    text_cam = f"{label} {obj_type} C: {P_cam[0]:.3f},{P_cam[1]:.3f},{P_cam[2]:.3f} m"
                    x, y = int(cx), int(cy)
                    cv2.putText(processed_frame, text_cam, (x + 6, y + 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 0), 1)

                    # camera -> base_link (via tool0 and TF) and store for publish
                    P_base = tf_helper.camera_to_base(P_cam)
                    if P_base is not None and label in poses_base:
                        poses_base[label].append(P_base)

            # Publish PoseArrays
            if tf_helper.ros_ok and publish_topics:
                now = tf_helper.rospy.Time.now()
                for label, arr in poses_base.items():
                    if len(arr) == 0:
                        continue
                    pa = PoseArray()
                    pa.header = Header(stamp=now, frame_id='base_link')
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
            cv2.imwrite(f"capture_{detection_mode}_detection.jpg", processed_frame)
            print(f"Saved image with {detection_mode} detection overlay.")
            break
        # Remove local mode switching; camera follows 'vision/mode' only

    cap.release()
    cv2.destroyAllWindows()



def detect_cans_on_frame(frame, px_to_m=None):
    """Enhanced can detection for drop zone. Detects green, blue, and gray cans.
    
    Returns: (image_with_overlays, detections)
    detection: {"label": str, "center_px": (cx, cy), "box": np.ndarray, "height": float}
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
    
    # Physical size filter: cans are ~7cm wide, ignore smaller than ~5cm
    # If px_to_m is available, convert 5 cm to pixels as a minimum diameter
    min_area = max(300, int(500 * (brightness / 255.0)))
    min_phys_diam_m = 0.05
    min_phys_area_px = None
    if px_to_m is not None and px_to_m > 0:
        min_diam_px = max(1.0, min_phys_diam_m / px_to_m)
        # approximate circular can footprint area threshold in pixels
        min_phys_area_px = np.pi * (0.5 * min_diam_px) ** 2
    
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
            if min_phys_area_px is not None and area_px < min_phys_area_px:
                continue
            hull = cv2.convexHull(contour)
            rect = cv2.minAreaRect(hull)
            (cx, cy), (w, h), _ = rect
            # Additional physical-size check on width/height if scale available
            if px_to_m is not None and px_to_m > 0:
                min_dim_m = min(w, h) * px_to_m
                required_min_dim_m = 0.06 if color_name == "blue" else min_phys_diam_m
                if min_dim_m < required_min_dim_m:
                    continue
            elif min(w, h) < 20:  # Fallback pixel threshold
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
            box = cv2.boxPoints(rect).astype(int)
            cv2.polylines(image, [box], True, (255, 165, 0), 2)  # Orange for cans
            cv2.circle(image, (int(cx), int(cy)), 6, (0, 0, 255), -1)
            cv2.putText(image, f"{color_name}_can", (box[0][0], box[0][1] - 6), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 165, 0), 1)
            
            # Estimate can height (10cm above A4 plane)
            can_height = 0.10  # 10cm as specified
            detections.append({
                "label": color_name,
                "center_px": (float(cx), float(cy)),
                "box": box,
                "height": can_height,
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

    # Tighter black range to reduce noise: require low V AND low S
    color_ranges = {
        "blue": ([90, min_sat, min_val], [130, 255, 255]),
        "green": ([35, min_sat, min_val], [85, 255, 255]),
        "black": ([0, 0, 0], [180, 70, 50]),  # S<=70, V<=50 to suppress dark wood/edges
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
        if color_name == "black":
            # Additional cleanup for black: erode small specks, then dilate back
            kernel_small = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
            mask = cv2.erode(mask, kernel_small, iterations=1)
            mask = cv2.dilate(mask, kernel_small, iterations=1)

        if brightness < 76.5:
            kernel_dilate = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            mask = cv2.dilate(mask, kernel_dilate, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue
            hull = cv2.convexHull(contour)
            rect = cv2.minAreaRect(hull)
            (cx, cy), (w, h), _ = rect
            if min(w, h) < 12:
                continue
            aspect = (w / h) if h > 1e-6 else 0
            if not 0.7 <= aspect <= 1.4:
                continue
            # Reject very irregular shapes (solidity check)
            hull2 = cv2.convexHull(contour)
            hull_area = max(cv2.contourArea(hull2), 1e-6)
            solidity = float(area) / hull_area
            if solidity < 0.80 and color_name == "black":
                continue
            box = cv2.boxPoints(rect).astype(int)
            cv2.polylines(image, [box], True, (0, 255, 0), 2)
            cv2.circle(image, (int(cx), int(cy)), 4, (0, 0, 255), -1)
            cv2.putText(image, color_name, (box[0][0], box[0][1] - 6), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            detections.append({
                "label": color_name,
                "center_px": (float(cx), float(cy)),
                "box": box,
            })

    return image, detections


if __name__ == "__main__":
    # Set width/height to your stream; if using Orbbec RGB, 1280x720 is typical.
    open_camera_with_transforms_dual_mode(0, width=1280, height=720, enable_ros_tf=True)


