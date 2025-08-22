## Introduction

This package is for usb camera extrinsics calibration, in the case of eye in hand structure.

## Prerequisite

1. Install hand-eye calibration ros package

```
mkdir -p ~/camera_ws/src
cd ~/camera_ws
git clone https://github.com/IFL-CAMP/easy_handeye.git ./src/
catkin_make
source devel/setup.bash
echo "source ~/camera_ws//devel/setup.bash" >> ~/.bashrc
```


2. Calibrate Camera Intrinsic Parameters

Refer [calibration guide](https://docs.sparklingrobo.com/xuanya-followerarm-teleop/intro-Alicia-D750-hand-eye-calibration-2)


## Usage

### Aruco Marker (Prefer)

Download and print aruco marker from [website](https://chev.me/arucogen/) with 20 mm size.

execute the following command
```
roslaunch alicia_d_calibration usb_aruco_eyeonhand.launch
```
Then follow the steps in [Documentation](https://docs.sparklingrobo.com/xuanya-followerarm-teleop/intro-Alicia-D750-hand-eye-calibration-2) to finish hand eye calibration procedure.


- Verification
In terminal one:
```
roslaunch alicia_d_driver alicia_d_bringup.launch
```
In terminal two:

```
roslaunch alicia_d_calibration usb_aruco_verify.launch
```
Add `TF` in RVIZ GUI to see if the camera_link frame is correct.


### Charuco Marker

Dowload and print charuco marker from [website](https://calib.io/pages/camera-calibration-pattern-generator?srsltid=AfmBOopueGksFnmCtSAtPeX8ks5lvzmnO9NrmYX4hH4ALVsXwqinzX2h)


execute the following command
```
roslaunch alicia_d_calibration usb_charuco_eyeonhand.launch
```

Then follow the steps in [Documentation](https://docs.sparklingrobo.com/xuanya-followerarm-teleop/intro-Alicia-D750-hand-eye-calibration-2) to finish hand eye calibration procedure.

- Verification
In terminal one:
```
roslaunch alicia_d_driver alicia_d_bringup.launch
```
In terminal two:

```
roslaunch alicia_d_calibration usb_charuco_verify.launch
```
Add `TF` in RVIZ GUI to see if the camera_link frame is correct.

