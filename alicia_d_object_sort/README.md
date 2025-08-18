# Alicia-D Object Sorting Package

## Introduction

This package provides 2D grasp and sorting capabilities for different colored cubes using the Alicia-D robotic manipulator. It combines computer vision for object detection with MoveIt-based motion planning for automated cube sorting operations.

## Prerequisites

Make sure you have completed the following steps:

1. **Robot MoveIt Control**: Alicia-D manipulator is properly configured and controlled via MoveIt
2. **USB Camera Calibration**: Both intrinsic and extrinsic calibration completed
   - Intrinsic calibration: Camera parameters (focal length, distortion coefficients)
   - Extrinsic calibration: Hand-eye calibration (camera to tool0 transformation)
  (Replace the `head_camera.yaml` and `usb_handeyecalibration_eye_on_hand.yaml` under `scripts` folder after calibration)
## Environment Setup

Prepare the following items:

- **Green and blue cubes** with 2 cm size
- **A4 paper sheet** for calibration reference
- **Calibrated USB camera** mounted on the robot end-effector
- **Alicia-D manipulator** with proper MoveIt configuration

## Installation

Install required Python dependencies:

```bash
# Install system packages
sudo apt install python3-opencv python3-yaml python3-numpy python3-matplotlib

# Install Python packages
pip3 install transforms3d
```

## Usage

### 1. Launch Robot Control

In terminal 1:
```bash
roslaunch alicia_d_driver alicia_d_bringup.launch
```

### 2. Start Object Detection

In terminal 2:
```bash
roscd alicia_d_object_sort/scripts
python3 camera_obj_detection.py
```

### 3. Start Cube Sorting

In terminal 3:
```bash
roscd alicia_d_object_sort/scripts
python3 cube_sorting.py
```

## Configuration

### Predefined Positions

You may need to modify the following predefined positions in `cube_sorting.py` to match your setup:

```python
# Predefined positions 
self.HOME_POSITION = [0.5101731659675737, -0.1741493363528409, 1.1561367836287713, 0.03145428542055679, -1.5228477209708768, -0.5163105875130477]
    
self.DROP_ZONE_POSITION = [1.4139084885387032, 0.08822543471619682, 0.33985971808065407, 0.046797839284243144, -1.2804195699246312, -1.3249158761293214]
self.DROP_ZONE_POSITION_2 = [1.3218471653565846, -0.24012661796669224, 0.8032350447639837, -0.026851219261451377, -1.4292520424023896, -1.1837551805834068]
```

**To find your desired pose:**
1. Use MoveIt's interactive markers to drag the robot end-effector
2. Record the corresponding robot joint state
3. Update the position values in the code

