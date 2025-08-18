#!/usr/bin/env python3
"""
Fixed ChArUco Detection Code for Jupyter Notebook

This code provides the corrected implementation that should replace
the failing cells in charuco.ipynb
"""

import cv2.aruco as aruco
import numpy as np
from Translation import Transformation
import time
import cv2
import matplotlib.pyplot as plt

# Camera calibration parameters
Dist = np.array([0.044021, -0.081267, -0.000946, 0.000040, 0.000000])  # system given

mtx=np.array([[531.14245,   0.     , 310.1948 ],
    [  0.     , 529.84709, 243.33563],
    [  0.     ,   0.     ,   1.     ]])


# Board parameters
squareLength = 0.015  # 15mm
markerLength = 0.011  # 11mm

# Load image
image = cv2.imread('captured_image.jpg')

print("=== FIXED CHARUCO DETECTION ===")

# Step 1: Detect ArUco markers
dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)

# Important: Disable corner refinement for ChArUco detection
detector_params = aruco.DetectorParameters()
detector_params.cornerRefinementMethod = aruco.CORNER_REFINE_NONE  # This is crucial!

detector = aruco.ArucoDetector(dictionary, detector_params)
marker_corners, marker_ids, rejected_candidates = detector.detectMarkers(image)

print(f'Detected {len(marker_ids) if marker_ids is not None else 0} ArUco markers')

if marker_ids is not None and len(marker_ids) > 0:
    # Step 2: Create ChArUco board with CORRECT dimensions and legacy pattern
    # The key fix: use (11, 8) dimensions and enable legacy pattern
    board = aruco.CharucoBoard((11, 8), squareLength, markerLength, dictionary)
    
    # CRITICAL: Enable legacy pattern for compatibility with older board formats
    board.setLegacyPattern(True)
    print("Legacy pattern enabled for OpenCV 4.x compatibility")
    
    # Step 3: Detect ChArUco corners
    charuco_detector = cv2.aruco.CharucoDetector(board, detectorParams=detector_params)
    charuco_corners, charuco_ids, marker_corners_found, marker_ids_found = charuco_detector.detectBoard(image)
    
    if charuco_ids is not None and len(charuco_ids) > 0:
        print(f'SUCCESS! Detected {len(charuco_ids)} ChArUco corners')
        
        # Step 4: Visualize the results
        image_copy = image.copy()
        
        # Draw detected ArUco markers
        aruco.drawDetectedMarkers(image_copy, marker_corners, marker_ids)
        
        # Draw detected ChArUco corners
        aruco.drawDetectedCornersCharuco(image_copy, charuco_corners, charuco_ids, (0, 255, 0))
        
        # Step 5: Estimate pose
        retval, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, board, mtx, Dist, None, None, useExtrinsicGuess=False
        )
        
        if retval:
            print("Pose estimation successful!")
            print(f"Rotation Vector (rvec): {rvec.flatten()}")
            print(f"Translation Vector (tvec): {tvec.flatten()}")
            
            # Draw coordinate axes
            cv2.drawFrameAxes(image_copy, mtx, Dist, rvec, tvec, 0.05)  # 5cm axes
            
        else:
            print("Pose estimation failed")
        
        # Display the result
        image_rgb = cv2.cvtColor(image_copy, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(12, 8))
        plt.imshow(image_rgb)
        plt.axis('off')
        plt.title(f'ChArUco Detection SUCCESS!\n{len(charuco_ids)} corners detected with 11x8 board + legacy pattern')
        plt.show()
        
        # Save the successful detection
        cv2.imwrite('charuco_detection_success.jpg', image_copy)
        print("Result saved as 'charuco_detection_success.jpg'")
        
        # Store results for further use
        all_charuco_corners = [charuco_corners]
        all_charuco_ids = [charuco_ids]
        
        print(f"\nSUMMARY:")
        print(f"- Board configuration: 11x8 with legacy pattern")
        print(f"- ArUco markers detected: {len(marker_ids)}")
        print(f"- ChArUco corners detected: {len(charuco_ids)}")
        print(f"- Pose estimation: {'SUCCESS' if retval else 'FAILED'}")
        
    else:
        print("ChArUco corner detection still failed")
        # Fallback: show just the ArUco markers
        image_with_markers = image.copy()
        aruco.drawDetectedMarkers(image_with_markers, marker_corners, marker_ids)
        
        image_rgb = cv2.cvtColor(image_with_markers, cv2.COLOR_BGR2RGB)
        plt.figure(figsize=(12, 8))
        plt.imshow(image_rgb)
        plt.axis('off')
        plt.title(f'ArUco Markers Only\n{len(marker_ids)} markers detected')
        plt.show()
        
else:
    print("No ArUco markers detected!")

print("\n=== EXPLANATION OF THE FIX ===")
print("The original failure was due to:")
print("1. Wrong board dimensions (should be 11x8, not 8x11)")
print("2. Missing legacy pattern flag for OpenCV 4.x compatibility")
print("3. Corner refinement enabled (should be disabled for ChArUco)")
print("4. These are common issues with recent OpenCV versions")
