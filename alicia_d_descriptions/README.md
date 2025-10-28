# Alicia-D Robot Descriptions

This package contains robot description files for the Alicia-D robots.

## Robot Models

The package uses the Synria-Robot-Descriptions submodule to access the latest robot URDF/MJCF models.

### Available Models

- **Alicia-D v5.5**: 100mm gripper
- **Alicia-D v5.6**: 50mm and 100mm grippers
- **Alicia-M v1.0**: 100mm gripper (cloud arm)

## Usage

### Display Robot Model in RViz

Launch the robot visualization with different versions and gripper types:

```bash
# Display v5.6 with 50mm gripper (default)
roslaunch alicia_d_descriptions display.launch

# Display v5.6 with 100mm gripper
roslaunch alicia_d_descriptions display.launch gripper_type:=100mm

# Display v5.5 with 100mm gripper
roslaunch alicia_d_descriptions display.launch robot_version:=v5_5 gripper_type:=100mm

# Enable joint state publisher GUI
roslaunch alicia_d_descriptions display.launch use_gui:=true

# Launch without RViz
roslaunch alicia_d_descriptions display.launch use_rviz:=false

```

### Launch Arguments

- `robot_version`: Robot version (`v5_5` or `v5_6`) - default: `v5_6`
- `gripper_type`: Gripper type (`50mm` or `100mm`) - default: `50mm`
- `use_gui`: Use joint_state_publisher GUI (`true` or `false`) - default: `false`
- `use_rviz`: Launch RViz (`true` or `false`) - default: `true`
- `use_fake_controllers`: Use fake joint state controllers (`true` or `false`) - default: `true`

## Submodule

The robot descriptions are managed via git submodule from:
- https://github.com/Synria-Robotics/Synria-Robot-Descriptions.git

To update the submodule:
```bash
cd alicia_d_descriptions
git submodule update --remote
```

