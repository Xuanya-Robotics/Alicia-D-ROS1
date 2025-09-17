import os
import sys
import time
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg
import numpy as np
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
# Add MoveIt script path to import search
THIS_DIR = os.path.dirname(__file__)
MOVEIT_SCRIPTS = os.path.abspath(os.path.join(THIS_DIR, '..', '..', 'alicia_d_moveit', 'scripts'))
if MOVEIT_SCRIPTS not in sys.path:
    sys.path.insert(0, MOVEIT_SCRIPTS)

from moveit_control import MoveItRobotController
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import String


class CubeSorting:
    def __init__(self):
        # Ensure this script is a ROS node (MoveItRobotController will also init if needed)
        if not rospy.get_node_uri():
            rospy.init_node('cube_sorting', anonymous=True)
        self.moveit_control = MoveItRobotController()
        self.q = quaternion_from_euler(0, np.pi, 0)
        # self.q = quaternion_from_euler(0, np.pi - 0.175, 0)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        # TF listener for base_link <-> tool0
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        # Load hand-eye (camera -> tool0)
        self.T_cam_tool0 = self._load_handeye_transform()
        
        # Subscribe to vision topics for cubes AND cans
        self.sub_cubes_green = rospy.Subscriber('vision/cubes/green', PoseArray, self._on_cubes_green)
        self.sub_cubes_blue  = rospy.Subscriber('vision/cubes/blue',  PoseArray, self._on_cubes_blue)
        self.sub_cubes_black = rospy.Subscriber('vision/cubes/black', PoseArray, self._on_cubes_black)
        # Ensure subscriptions exist for cans even if camera node starts in cube mode
        self.sub_cans_green = rospy.Subscriber('vision/cans/green', PoseArray, self._on_cans_green)
        self.sub_cans_blue  = rospy.Subscriber('vision/cans/blue',  PoseArray, self._on_cans_blue)
        self.sub_cans_red   = rospy.Subscriber('vision/cans/red',   PoseArray, self._on_cans_red)
        
        # Storage for detected objects
        self.latest_cubes = {'green': [], 'blue': [], 'black': []}
        self.latest_cans = {'green': [], 'blue': [], 'red': []}
        # Detection mode publisher (camera subscribes)
        self.mode_pub = rospy.Publisher('vision/mode', String, queue_size=1, latch=True)
        
        # Workflow state management
        # self.workflow_state = "moving_to_drop"  # idle, cube_detection, moving_to_pickup, moving_to_drop, can_detection
        self.workflow_state = "idle"  # idle, cube_detection, moving_to_pickup, moving_to_drop, can_detection
        self.current_cube_color = None
        self.current_cube_position = None
        self._moving_now = False
        
        # Predefined positions 
        self.HOME_POSITION = [0.5101731659675737, -0.1741493363528409, 1.1561367836287713, 0.03145428542055679, -1.5228477209708768, -0.5163105875130477]
            
        self.DROP_ZONE_POSITION = [1.3095723222656355, -0.048332194670612276, 0.830853441718619, -0.05446961621608682, -1.4829544809252917, -1.3617404054021693]

# Removed old grasp_cube method - replaced by _select_and_pick_cube in workflow
    
    def _on_cubes_green(self, msg: PoseArray):
        self.latest_cubes['green'] = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]

    def _on_cubes_blue(self, msg: PoseArray):
        self.latest_cubes['blue'] = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]

    def _on_cubes_black(self, msg: PoseArray):
        self.latest_cubes['black'] = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]
        
    def _on_cans_green(self, msg: PoseArray):
        self.latest_cans['green'] = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]

    def _on_cans_blue(self, msg: PoseArray):
        self.latest_cans['blue'] = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]

    def _on_cans_red(self, msg: PoseArray):
        self.latest_cans['red'] = [(p.position.x, p.position.y, p.position.z) for p in msg.poses]

    def execute_cube_sorting_workflow(self):
        """Complete cube sorting workflow with state management."""
        if self._moving_now:
            return
            
        if self.workflow_state == "idle":
            # Start by moving to home position for cube detection
            rospy.loginfo("Starting cube sorting workflow - moving to home position")
            self.move_to_home()
            self.moveit_control.open_gripper()
            # Switch to cubes mode via topic
            self._set_mode('cubes')
            self.workflow_state = "cube_detection"
            rospy.loginfo("Moved to home. Please start cube detection (cubes mode)")
            
        elif self.workflow_state == "cube_detection":
            # Wait for cube detection and select a cube to pick
            time.sleep(2)
            if self._select_and_pick_cube():
                self.workflow_state = "moving_to_drop"
        elif self.workflow_state == "moving_to_drop":
            # Move to drop zone for can detection
            self._move_to_drop_zone()
            # Switch to cans mode via topic
            self._set_mode('cans')
            self.workflow_state = "idle"
            # self.workflow_state = "can_detection"
            rospy.loginfo("Moved to drop zone. Please start can detection (cans mode)")
            
        # elif self.workflow_state == "can_detection":
        #     # Wait for can detection and drop cube in appropriate can
        #     if self._drop_cube_in_can():
        #         self.workflow_state = "idle"
                
        # elif self.workflow_state == "returning_home":
        #     # Return to home position and reset for next cube
        #     self.move_to_home()
        #     self.workflow_state = "cube_detection"
        #     rospy.loginfo("Returned to home. Ready for next cube.")

    def _set_mode(self, mode: str):
        mode = (mode or '').strip().lower()
        if mode not in ('cubes', 'cans'):
            return
        self.mode_pub.publish(String(data=mode))
        
    def _clear_cube_detections(self):
        """Clear all stored cube detections to get fresh data."""
        rospy.loginfo("Clearing old cube detections for fresh detection cycle")
        self.latest_cubes = {'green': [], 'blue': [], 'black': []}
        
    def _clear_can_detections(self):
        """Clear all stored can detections to get fresh data."""
        rospy.loginfo("Clearing old can detections for fresh detection cycle")
        self.latest_cans = {'green': [], 'blue': [], 'red': []}
            
    def _select_and_pick_cube(self):
        """Select and pick up a cube. Returns True if successful."""
        # Wait a bit to accumulate fresh detections
        rospy.loginfo("Waiting for fresh cube detections...")
        time.sleep(3)
        
        # Choose target by priority: green, blue, black
        target = None
        color = None
        for c in ['green', 'blue', 'black']:
            if self.latest_cubes[c]:
                target = self.latest_cubes[c].pop(0)  # Remove from list to prevent re-picking
                color = c
                break
                
        if target is None:
            rospy.loginfo('No cubes available to pick. Waiting for detections...')
            return False
            
        rospy.loginfo(f"Picking {color} cube at {target}")
        self.current_cube_color = color
        self.current_cube_position = target
        
        # Clear remaining detections of the same color to prevent confusion
        # if color in self.latest_cubes:
        #     remaining_count = len(self.latest_cubes[color])
        #     if remaining_count > 0:
        #         rospy.loginfo(f"Clearing {remaining_count} remaining {color} cube detections to prevent confusion")
        #         self.latest_cubes[color] = []
        
        self._moving_now = True
        try:
            # Move to pre-grasp above target
            pre = Pose()
            pre.position.x, pre.position.y, pre.position.z = target[0], target[1], target[2] + 0.05
            q = quaternion_from_euler(0, np.pi, 0)
            pre.orientation.x, pre.orientation.y, pre.orientation.z, pre.orientation.w = q
            success = self.moveit_control.move_to_pose(pre)
            if not success:
                rospy.logwarn("Failed to move to pre-grasp position")
                return False
            time.sleep(2)
            # Approach and grasp (close gripper via MoveIt or your driver)
            grasp = Pose()
            grasp.position.x, grasp.position.y, grasp.position.z = target[0], target[1], target[2] + 0.03
            grasp.orientation.x, grasp.orientation.y, grasp.orientation.z, grasp.orientation.w = q
            success = self.moveit_control.move_to_pose(grasp)
            self.moveit_control.close_gripper(0.035)
            time.sleep(2)
            if not success:
                rospy.logwarn("Failed to move to grasp position")
                return False

            
            # Lift cube slightly after grasping
            lift = Pose()
            lift.position.x, lift.position.y, lift.position.z = target[0], target[1], target[2] + 0.1
            lift.orientation.x, lift.orientation.y, lift.orientation.z, lift.orientation.w = q
            success = self.moveit_control.move_to_pose(lift)
            if not success:
                rospy.logwarn("Failed to lift cube")
                return False
                
            return True
        except Exception as e:
            rospy.logerr(f"Error during cube pickup: {e}")
            return False
        finally:
            self._moving_now = False
            
    def _move_to_drop_zone(self):
        """Move to drop zone position for can detection."""
        # rospy.loginfo("Moving to drop zone position")
        self._moving_now = True
        try:
            # self.validate_joint_limits(self.DROP_ZONE_POSITION)
            success = self.moveit_control.move_to_joint_state(self.DROP_ZONE_POSITION)
            gripper_success = self.moveit_control.open_gripper()
            self._clear_cube_detections()
            time.sleep(2)
            
            # time.sleep(2)
            if not success:
                rospy.logwarn("Failed to move to drop zone position")
            return success
        finally:
            self._moving_now = False
            
    def _drop_cube_in_can(self):
        """Drop cube in the appropriate can. Returns True if successful."""
        if self.current_cube_color is None:
            rospy.logwarn("No cube color information available")
            return False
        
        # Wait for fresh can detections
        rospy.loginfo("Waiting for fresh can detections...")
        time.sleep(3)
        
        can_color_map = {
            'green': 'green',
            'blue': 'blue',
            'black': 'red'  
        }
        target_can_color = can_color_map.get(self.current_cube_color)
        
        # Require a minimum number of detections to stabilize selection
        if target_can_color not in self.latest_cans or len(self.latest_cans[target_can_color]) == 0:
            rospy.logwarn(f"No {target_can_color} can detected for {self.current_cube_color} cube")
            return False
            
        can_position = self.latest_cans[target_can_color][0]  # Take first detected can
        rospy.loginfo(f"Dropping {self.current_cube_color} cube in {target_can_color} can at {can_position}")
        
        # Clear can detections to get fresh data for next cycle
        self._clear_can_detections()
        
        
        # target = None
        # can_position = None
        # if self.latest_cans[color]:
        #     can_position = self.latest_cans[color].pop(0)

        # if can_position is None:
        #     rospy.loginfo('No {color} cans available to drop.')
        #     return False
        
        # rospy.loginfo(f"Dropping {color} cube at {can_position}")
        # self.current_cube_color = color
        # self.current_cube_position = target

        self._moving_now = True
        try:
            # Move to pre-drop position above can
            pre_drop = Pose()
            pre_drop.position.x, pre_drop.position.y, pre_drop.position.z = can_position[0], can_position[1], can_position[2] + 0.1
            q = quaternion_from_euler(0, np.pi, 0)
            pre_drop.orientation.x, pre_drop.orientation.y, pre_drop.orientation.z, pre_drop.orientation.w = q
            success = self.moveit_control.move_to_pose(pre_drop)
            if not success:
                rospy.logwarn("Failed to move to drop position")
                return False
            # open gripper
            self.moveit_control.open_gripper()
            time.sleep(2)

            # Clear current cube info
            self.current_cube_color = None
            self.current_cube_position = None
            rospy.loginfo("current_cube_color after drop: ", self.current_cube_color)
            
            return True
        except Exception as e:
            rospy.logerr(f"Error during cube drop: {e}")
            return False
        finally:
            self._moving_now = False




    def get_pose(self):
        return self.moveit_control.get_current_pose()

    def get_state(self):
        return self.moveit_control.get_current_joint_state()
    
    def validate_joint_limits(self, joint_values):
        """Validate that joint values are within robot limits"""
        # Joint limits from URDF: [lower, upper]
        limits = [
            [-2.16, 2.16],    # Joint1
            [-1.57, 1.57],    # Joint2
            [-0.5, 2.35619],  # Joint3
            [-3.14, 3.14],    # Joint4
            [-1.57, 1.5],     # Joint5
            [-3.14, 3.14]     # Joint6
        ]
        
        if len(joint_values) != len(limits):
            raise ValueError(f"Expected {len(limits)} joint values, got {len(joint_values)}")
        
        for i, (value, (lower, upper)) in enumerate(zip(joint_values, limits)):
            if not (lower <= value <= upper):
                raise ValueError(f"Joint{i+1} value {value:.4f} is outside limits [{lower}, {upper}]")
        
        return True

    def move_to_home(self):
        """Move robot to home position for cube detection."""
        rospy.loginfo("Moving to home position for cube detection")
        # Validate joint limits before moving
        self.validate_joint_limits(self.HOME_POSITION)
        success = self.moveit_control.move_to_joint_state(self.HOME_POSITION)
        time.sleep(2)
        if not success:
            rospy.logwarn("Failed to move to home position")
        return success
        
    def reset_workflow(self):
        """Reset workflow state to idle."""
        self.workflow_state = "idle"
        self.current_cube_color = None
        self.current_cube_position = None
        self._moving_now = False
        # Clear all detections for fresh start
        self._clear_cube_detections()
        self._clear_can_detections()
        rospy.loginfo("Workflow reset to idle state with cleared detections")
        
    def force_clear_detections(self):
        """Manually clear all detection data - useful for debugging."""
        self._clear_cube_detections()
        self._clear_can_detections()
        rospy.loginfo("Manually cleared all detection data")
        
    def get_workflow_status(self):
        """Get current workflow status."""
        return {
            'state': self.workflow_state,
            'current_cube_color': self.current_cube_color,
            'current_cube_position': self.current_cube_position,
            'moving': self._moving_now,
            'cubes_detected': {k: len(v) for k, v in self.latest_cubes.items()},
            'cans_detected': {k: len(v) for k, v in self.latest_cans.items()},
            'total_cubes': sum(len(v) for v in self.latest_cubes.values()),
            'total_cans': sum(len(v) for v in self.latest_cans.values())
        }



    def _load_handeye_transform(self):
        """Load T_cam_tool0 from yaml. Returns 4x4 np.array or None."""
        yaml_path = os.path.join(os.path.dirname(__file__), 'usb_handeyecalibration_eye_on_hand.yaml')
        if not os.path.exists(yaml_path):
            return None
        import yaml
        with open(yaml_path, 'r') as f:
            data = yaml.safe_load(f)
        t = data.get('transformation', {})
        qw = float(t.get('qw', 1.0)); qx = float(t.get('qx', 0.0)); qy = float(t.get('qy', 0.0)); qz = float(t.get('qz', 0.0))
        tx = float(t.get('x', 0.0)); ty = float(t.get('y', 0.0)); tz = float(t.get('z', 0.0))
        q = np.array([qw, qx, qy, qz], dtype=np.float64)
        q /= np.linalg.norm(q)
        R = tf_conversions.transformations.quaternion_matrix([q[1], q[2], q[3], q[0]])
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R[:3, :3]
        T[:3, 3] = np.array([tx, ty, tz], dtype=np.float64)
        return T

    def camera_to_tool(self, Pc):
        """Pc: 3-vector in camera frame -> tool0 frame (3-vector)."""
        if self.T_cam_tool0 is None:
            raise RuntimeError('Hand-eye transform not loaded')
        Pc_h = np.array([Pc[0], Pc[1], Pc[2], 1.0], dtype=np.float64)
        Pt_h = self.T_cam_tool0 @ Pc_h
        return Pt_h[:3]

    def camera_to_base(self, Pc):
        """Pc: 3-vector in camera frame -> base_link frame (3-vector)."""
        Pt = self.camera_to_tool(Pc)
        try:
            tr = self.tf_buffer.lookup_transform('base_link', 'tool0', rospy.Time(0), rospy.Duration(0.1))
            t = tr.transform.translation
            r = tr.transform.rotation
            Tbt = tf_conversions.transformations.quaternion_matrix([r.x, r.y, r.z, r.w])
            Tbt[0, 3] = t.x; Tbt[1, 3] = t.y; Tbt[2, 3] = t.z
            Pt_h = np.array([Pt[0], Pt[1], Pt[2], 1.0], dtype=np.float64)
            Pb_h = Tbt @ Pt_h
            return Pb_h[:3]
        except Exception:
            # Fallback to tool frame if TF not available
            return Pt

if __name__ == "__main__":
    import time
    
    # Create cube sorting controller
    cube_sorting = CubeSorting()
    
    # Set up workflow mode
    WORKFLOW_MODE = True  # Set to True for automated workflow, False for manual testing
    
    if WORKFLOW_MODE:
        rospy.loginfo("Starting automated cube sorting workflow...")
        rospy.loginfo("Instructions:")
        rospy.loginfo("1. Robot will move to home position for cube detection")
        rospy.loginfo("2. Start camera_a4_to_base.py in 'cubes' mode to detect cubes")
        rospy.loginfo("3. Robot will pick up cubes and move to drop zone")
        rospy.loginfo("4. Switch camera to 'cans' mode to detect placement cans")
        rospy.loginfo("5. Robot will drop cubes in appropriate cans")
        rospy.loginfo("")
        rospy.loginfo("Press Ctrl+C to stop")
        
        rate = rospy.Rate(1)  # 1 Hz
        try:
            while not rospy.is_shutdown():
                # Execute one step of the workflow
                cube_sorting.execute_cube_sorting_workflow()
                
                # Print status every few seconds
                # status = cube_sorting.get_workflow_status()
                # rospy.loginfo_throttle(5, f"Workflow Status: {status}")
                
                rate.sleep()
        except KeyboardInterrupt:
            rospy.loginfo("Cube sorting workflow stopped by user")
    else:
        # Manual testing mode
        rospy.loginfo("Manual testing mode - robot will move to home position")
        cube_sorting.move_to_home()
        rospy.loginfo("Manual testing ready. Use cube_sorting object to test individual functions.")
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass