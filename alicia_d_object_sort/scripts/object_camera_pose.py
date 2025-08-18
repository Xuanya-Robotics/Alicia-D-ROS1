import cv2
import numpy as np
import os
try:
    import yaml
except Exception:
    yaml = None

def order_points_clockwise(pts):
    pts = np.array(pts, dtype=np.float32)
    c = pts.mean(axis=0)
    angles = np.arctan2(pts[:, 1] - c[1], pts[:, 0] - c[0])
    order = np.argsort(angles)
    pts = pts[order]
    # Ensure consistent TL, TR, BR, BL ordering (start from the top-left)
    s = pts.sum(axis=1)
    tl_idx = np.argmin(s)
    pts = np.roll(pts, -tl_idx, axis=0)
    return pts


def estimate_pose_from_square(image, image_points, square_size_m, camera_matrix, dist_coeffs):
    # image_points: 4x2, ordered TL, TR, BR, BL
    half = square_size_m / 2.0
    object_points = np.array([
        [-half, -half, 0.0],
        [ half, -half, 0.0],
        [ half,  half, 0.0],
        [-half,  half, 0.0],
    ], dtype=np.float32)

    img_pts = np.array(image_points, dtype=np.float32).reshape(-1, 1, 2)
    obj_pts = object_points.reshape(-1, 1, 3)

    success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not success:
        return None

    # Compute cube center in camera coords: center = t - (size/2) * R[:,2]
    R, _ = cv2.Rodrigues(rvec)
    center_cam = (tvec.reshape(3) - half * R[:, 2]).reshape(3, 1)

    # Draw axes for visualization
    axis_len = square_size_m * 1.5
    axes_3d = np.float32([[0, 0, 0], [axis_len, 0, 0], [0, axis_len, 0], [0, 0, axis_len]]).reshape(-1, 1, 3)
    img_axes, _ = cv2.projectPoints(axes_3d, rvec, tvec, camera_matrix, dist_coeffs)
    img_axes = img_axes.reshape(-1, 2).astype(int)
    origin = tuple(img_axes[0])
    cv2.line(image, origin, tuple(img_axes[1]), (0, 0, 255), 2)  # X - red
    cv2.line(image, origin, tuple(img_axes[2]), (0, 255, 0), 2)  # Y - green
    cv2.line(image, origin, tuple(img_axes[3]), (255, 0, 0), 2)  # Z - blue

    return {
        "rvec": rvec,
        "tvec_face_center": tvec,
        "cube_center_cam": center_cam,
        "R": R,
    }


def detect_a4_rectangle(image_bgr, min_area=20000):
    """
    Detect the largest white rectangle (A4 paper) and return its 4 corners
    ordered as TL, TR, BR, BL. Returns None if not found.
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    # White paper: low saturation, high value
    mask_white = cv2.inRange(hsv, np.array([0, 0, 180], np.uint8), np.array([179, 70, 255], np.uint8))
    # Also use grayscale threshold to reinforce
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    _, mask_gray = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    mask = cv2.bitwise_and(mask_white, mask_gray)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7)))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)))

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cnt = max(contours, key=cv2.contourArea)
        if cv2.contourArea(cnt) >= min_area:
            hull = cv2.convexHull(cnt)
            eps = 0.02 * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, eps, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                corners = approx.reshape(-1, 2)
                return order_points_clockwise(corners)
            else:
                corners = cv2.boxPoints(cv2.minAreaRect(hull))
                return order_points_clockwise(corners)

    # Fallback: edge-based largest quadrilateral
    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    edges = cv2.dilate(edges, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1)
    cnts, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_area = 0
    for c in cnts:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        eps = 0.02 * cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, eps, True)
        if len(approx) == 4 and cv2.isContourConvex(approx) and area > best_area:
            best = approx
            best_area = area
    if best is not None:
        return order_points_clockwise(best.reshape(-1, 2))
    return None


def estimate_plane_from_rectangle(corners_img, rect_w_m, rect_h_m, K, dist):
    """
    corners_img: 4x2 TL, TR, BR, BL in pixels.
    rect_w_m, rect_h_m: physical size of rectangle (meters) along TL->TR and TR->BR.
    Returns plane normal n (3x1), point p0 on plane (3x1), and pose (rvec, tvec).
    """
    obj = np.array([
        [0, 0, 0],
        [rect_w_m, 0, 0],
        [rect_w_m, rect_h_m, 0],
        [0, rect_h_m, 0],
    ], dtype=np.float32).reshape(-1, 1, 3)
    img = np.array(corners_img, dtype=np.float32).reshape(-1, 1, 2)

    # Choose a robust flag (square vs rectangle)
    flag = cv2.SOLVEPNP_IPPE_SQUARE if abs(rect_w_m - rect_h_m) < 1e-6 else cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=flag)
    if not ok:
        return None
    if hasattr(cv2, "solvePnPRefineLM"):
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    R, _ = cv2.Rodrigues(rvec)
    n = R[:, 2:3]
    p0 = tvec
    return n, p0, (rvec, tvec)


def pixel_to_plane_point(u, v, K, n, p0):
    ray = np.linalg.inv(K) @ np.array([u, v, 1.0], dtype=np.float32)
    ray = ray / np.linalg.norm(ray)
    denom = float(n.T @ ray.reshape(3, 1))
    if abs(denom) < 1e-8:
        return None
    s = float(n.T @ p0) / denom
    return (s * ray).reshape(3)


def load_intrinsics_from_yaml(yaml_path, image_shape):
    """
    Load camera intrinsics from a ROS-style YAML file and scale to the current image size.
    Returns K (3x3 float32) and dist (1x5 float32).
    """
    H, W = image_shape[:2]
    if yaml is None:
        raise RuntimeError("PyYAML not available. Please install pyyaml or provide calib.npz.")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    W_yaml = float(data.get('image_width', W))
    H_yaml = float(data.get('image_height', H))
    K_list = data['camera_matrix']['data']
    D_list = data['distortion_coefficients']['data']
    K = np.array(K_list, dtype=np.float32).reshape(3, 3)
    dist = np.array(D_list, dtype=np.float32).reshape(1, -1)

    # Scale intrinsics if resolution differs
    sx = W / W_yaml
    sy = H / H_yaml
    K[0, 0] *= sx
    K[0, 2] *= sx
    K[1, 1] *= sy
    K[1, 2] *= sy
    return K, dist
    
def validate_intrinsics_with_a4(corners_img, K, dist, A4_W=0.297, A4_H=0.210):
    obj = np.array([[0,0,0],[A4_W,0,0],[A4_W,A4_H,0],[0,A4_H,0]], np.float32).reshape(-1,1,3)
    img = np.array(corners_img, np.float32).reshape(-1,1,2)
    flag = cv2.SOLVEPNP_ITERATIVE
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=flag)
    if not ok: 
        print("PnP failed for A4.")
        return
    reproj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    err = np.linalg.norm(reproj.reshape(-1,2) - img.reshape(-1,2), axis=1)
    print(f"Reprojection error px (mean/max): {err.mean():.3f} / {err.max():.3f}")


def detect_colored_cubes(image_path):
    """
    Detects blue and green cubes in an image, draws bounding boxes,
    and calculates their centers.
    """
    image = cv2.imread(image_path)
    if image is None:
        print(f"Error: Could not load image from {image_path}")
        return

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Stabilize brightness/contrast for more consistent masks
    h, s, v = cv2.split(hsv_image)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    v = clahe.apply(v)
    hsv_image = cv2.merge([h, s, v])

    color_ranges = {
        "blue": ([90, 100, 100], [130, 255, 255]),
        "green": ([35, 80, 80], [85, 255, 255]),
    }

    min_area = 200

    # Camera intrinsics from YAML (head_camera.yaml); fallback to simple guess
    yaml_path = os.path.join(os.path.dirname(__file__), 'head_camera.yaml')
    if os.path.exists(yaml_path) and yaml is not None:
        camera_matrix, dist_coeffs = load_intrinsics_from_yaml(yaml_path, image.shape)
        # Undistort and use rectified intrinsics
        h, w = image.shape[:2]
        newK, _ = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 0)
        image = cv2.undistort(image, camera_matrix, dist_coeffs, None, newK)
        camera_matrix = newK.astype(np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        h, s, v = cv2.split(hsv_image)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        v = clahe.apply(v)
        hsv_image = cv2.merge([h, s, v])
    else:
        camera_matrix = np.array([[600.0, 0.0, image.shape[1] / 2.0],
                                  [0.0, 600.0, image.shape[0] / 2.0],
                                  [0.0,   0.0,                 1.0]], dtype=np.float32)
        dist_coeffs = np.zeros((5, 1), dtype=np.float32)
    cube_size_m = 0.02

    # Optional: detect A4 plane (roughly 210mm x 297mm). Choose orientation you laid the paper.
    # Here we assume long edge aligned TL->TR, so width=0.297m, height=0.210m.
    # If your paper orientation is swapped, flip these numbers.
    A4_W, A4_H = 0.297, 0.210
    a4_corners = detect_a4_rectangle(image)
    plane = None
    if a4_corners is not None:
        # Subpixel refine and auto-orient A4 rectangle
        gray_ref = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-4)
        corners_fp = a4_corners.astype(np.float32).reshape(-1, 1, 2)
        corners_ref = cv2.cornerSubPix(gray_ref, corners_fp, (7, 7), (-1, -1), term).reshape(-1, 2)
        # pick orientation based on longer pixel edge
        w_px = np.linalg.norm(corners_ref[1] - corners_ref[0])
        h_px = np.linalg.norm(corners_ref[2] - corners_ref[1])
        if h_px > w_px:
            A4_W, A4_H = 0.210, 0.297
        plane = estimate_plane_from_rectangle(corners_ref, A4_W, A4_H, camera_matrix, dist_coeffs)
        a4_corners = corners_ref
        # Draw outline and corners with labels TL, TR, BR, BL for correctness check
        a4_i = a4_corners.astype(int)
        cv2.polylines(image, [a4_i], True, (0, 255, 255), 3)
        labels = ["TL", "TR", "BR", "BL"]
        colors = [(0,0,255), (0,255,0), (255,0,0), (0,255,255)]
        for idx, (pt, lab, col) in enumerate(zip(a4_i, labels, colors)):
            x, y = int(pt[0]), int(pt[1])
            cv2.circle(image, (x, y), 6, col, -1)
            cv2.putText(image, f"{lab}", (x+6, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

    for color_name, (lower_bound, upper_bound) in color_ranges.items():
        lower = np.array(lower_bound, dtype="uint8")
        upper = np.array(upper_bound, dtype="uint8")

        # Build a clean mask (denoise + open/close to remove speckles and fill small gaps)
        mask = cv2.inRange(hsv_image, lower, upper)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        kernel_open = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        kernel_close = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel_open, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            # Use convex hull for stability and filter by solidity
            hull = cv2.convexHull(contour)
            hull_area = cv2.contourArea(hull)
            if hull_area <= 0:
                continue
            solidity = area / hull_area
            if solidity < 0.9:
                continue

            # Tight, rotated bounding box around the object
            rect = cv2.minAreaRect(hull)            # ((cx, cy), (w, h), angle)
            (cx, cy), (w, h), angle = rect
            if min(w, h) < 15:                      # ignore tiny leftovers
                continue
            aspect = (w / h) if h > 1e-6 else 0
            if not 0.8 <= aspect <= 1.25:           # roughly square
                continue

            # Prefer a 4-point polygon fit for tighter boxes; fall back to minAreaRect
            epsilon = 0.03 * cv2.arcLength(hull, True)
            approx = cv2.approxPolyDP(hull, epsilon, True)
            if len(approx) == 4 and cv2.isContourConvex(approx):
                box = approx.reshape(-1, 2)
                ordered = order_points_clockwise(box)
                cX, cY = ordered.mean(axis=0)
                # Pose estimation from square
                pose = estimate_pose_from_square(image, ordered, cube_size_m, camera_matrix, dist_coeffs)
            else:
                box = cv2.boxPoints(rect)
                ordered = order_points_clockwise(box)
                cX, cY = cx, cy
                pose = estimate_pose_from_square(image, ordered, cube_size_m, camera_matrix, dist_coeffs)

            box_i = box.astype(int)
            cv2.polylines(image, [box_i], True, (0, 255, 0), 2)

            # Center from the chosen polygon
            cX, cY = int(round(cX)), int(round(cY))
            cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
            # 3D from plane intersection if A4 detected
            if plane is not None:
                n, p0, _ = plane
                P = pixel_to_plane_point(float(cX), float(cY), camera_matrix, n, p0)
                if P is not None:
                    print(f"Detected '{color_name}' at (x,y)=({cX},{cY}); 3D on A4 plane (m): {P}")
            elif pose is not None:
                center_cam = pose["cube_center_cam"].reshape(-1)
                print(f"Detected '{color_name}' at (x,y)=({cX},{cY}); 3D from PnP (m): {center_cam}")
            else:
                print(f"Detected '{color_name}' at (x,y)=({cX},{cY})")
            cv2.putText(image, color_name, (box_i[0][0], box_i[0][1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    cv2.imshow("Detected Cubes with Centers", image)
    print("\nDetections complete. Press any key to exit.")
    cv2.waitKey(0)
    output_image_path = 'detected_centers1.jpg'
    cv2.imwrite(output_image_path, image)
    print(f"Image with centers saved to {output_image_path}")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    IMAGE_FILE = 'captured_image.jpg'  # Make sure this is your image file

    # Optional: validate intrinsics using A4 corners
    img = cv2.imread(IMAGE_FILE)
    if img is not None:
        yaml_path = os.path.join(os.path.dirname(__file__), 'head_camera.yaml')
        if os.path.exists(yaml_path) and yaml is not None:
            K, dist = load_intrinsics_from_yaml(yaml_path, img.shape)
            a4 = detect_a4_rectangle(img)
            if a4 is not None:
                print(f"A4 detected corners (px): {a4.tolist()}")
                validate_intrinsics_with_a4(a4, K, dist)
            else:
                print("A4 paper not detected for intrinsics validation.")

    # Run detection + 3D estimation (script loads intrinsics internally)
    detect_colored_cubes(IMAGE_FILE)
