# Alicia-D ROS1 Basic Usage Guide

This document provides basic usage instructions and examples for Alicia-D ROS1 (Noetic).

## Standalone Driver Node (No MoveIt)

### Launch

```bash
roslaunch alicia_d_driver alicia_d_driver.launch
```

### Read states

```bash
rostopic echo /joint_states
```

### Subscribed / Published Topics (Driver)

- Subscribed:
  - `/joint_commands` (sensor_msgs/JointState)
  - `/demonstration` (std_msgs/Bool)
  - `/zero_calibrate` (std_msgs/Bool)
  - `/default_speed_deg_s` (std_msgs/Float64) - update driver default speed at runtime (deg/s)
- Published:
  - `/joint_states` (sensor_msgs/JointState)

### Enable / Disable Hand-Guiding Mode (Zero Torque)

Enable:

```bash
rostopic pub -1 /demonstration std_msgs/Bool "data: true"
```

Disable:

```bash
rostopic pub -1 /demonstration std_msgs/Bool "data: false"
```

### Zero Calibration

⚠️ Warning: This operation is irreversible. Skip this step if not necessary.

Step 1: Disable torque first (enter hand-guiding mode)

```bash
rostopic pub -1 /demonstration std_msgs/Bool "data: true"
```

Step 2: Manually move the robot to the desired zero position

Step 3: Execute zero calibration (torque will be automatically restored after calibration)

```bash
rostopic pub -1 /zero_calibrate std_msgs/Bool "data: true"
```

### Send Joint Commands

Send joint position and (optional) velocity commands via `/joint_commands`:

Example 1: Move to a pose with default speed (driver `default_speed_deg_s`)

```bash
rostopic pub -1 /joint_commands sensor_msgs/JointState "
name: ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Gripper']
position: [0.5, 0.1, 0.0, 0.0, 0.1, 0.0, 500.0]
"
```

Example 2: Move with a specific speed cap (30 deg/s = 0.524 rad/s)

```bash
rostopic pub -1 /joint_commands sensor_msgs/JointState "
name: ['Joint1','Joint2','Joint3','Joint4','Joint5','Joint6','Gripper']
position: [0.5, 0.1, 0.0, 0.0, 0.1, 0.0, 500.0]
velocity: [0.524, 0.524, 0.524, 0.524, 0.524, 0.524, 0.0]
"
```

#### Note on Units (Important)

`/joint_commands` uses `sensor_msgs/JointState` (ROS standard):

- Positions are in **radians (rad)**
- Velocities in `velocity` are **radians per second (rad/s)**

Driver behavior:

- If `velocity` is provided (rad/s), the driver converts it to deg/s internally and uses the **maximum** as a common speed
- If `velocity` is not provided, the driver uses its `default_speed_deg_s` (deg/s)
- Gripper command in `/joint_commands` is **raw 0..1000** (0=closed, 1000=open)

Speed conversion reference:

- 20 deg/s = 0.349 rad/s
- 30 deg/s = 0.524 rad/s
- 40 deg/s = 0.698 rad/s
- 100 deg/s = 1.745 rad/s

## Drag Teaching (Record / Replay)

Launch:

```bash
roslaunch alicia_d_drag_teaching drag_teaching.launch
```

Motions are saved to:

- `$(rospack find alicia_d_drag_teaching)/example_motions/<motion_name>/`

Common usage:

- Record (auto):
  - `mode:=auto save_motion:=my_motion`
- Replay only:
  - `mode:=replay_only save_motion:=my_motion`
- Replay speed control:
  - `replay_speed_deg_s:=0.0` → replay uses recorded timing (`t`)
  - `replay_speed_deg_s:=20.0` → cap replay speed (deg/s) by publishing `JointState.velocity`

Update driver default speed at runtime (deg/s):

```bash
rostopic pub -1 /default_speed_deg_s std_msgs/Float64 "data: 20.0"
```
