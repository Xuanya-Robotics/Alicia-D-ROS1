#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS Drag Teaching 

Modes:
- manual: torque off, user presses Enter to record keypoints
- auto: torque off, sample joint states continuously at a target rate
- replay_only: load saved motion and replay it by publishing /joint_commands

Data format (compatible in spirit with SDK):
- example_motions/<motion>/joint_traj.json : list of {"t": float, "q": [..], "grip": float}
- example_motions/<motion>/meta.json : metadata

Notes:
- Reads robot state from /joint_states (JointState)
- Controls drag mode by publishing /demonstration (Bool): True=torque off, False=torque on
- Replays by publishing /joint_commands (JointState).
  - Positions: arm joints in rad, gripper in **raw 0..1000** (driver semantics)
  - If JointState.velocity is provided (rad/s), driver converts it to deg/s and uses max(abs(v)) as speed cap
  - If no velocity is provided, driver uses its `default_speed_deg_s`
"""

import argparse
import json
import os
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime
from typing import Dict, List, Optional

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_msgs.msg import Float64


ARM_JOINTS = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6"]
GRIPPER_JOINT = "Gripper"


def stroke_m_from_type(gripper_type: str) -> float:
    return 0.05 if gripper_type == "100mm" else 0.025


def gripper_value_to_position_m(value_0_1000: float, gripper_type: str) -> float:
    # Keep existing driver semantics: value 0=closed, 1000=open; position 0=open, stroke=closed
    stroke_m = stroke_m_from_type(gripper_type)
    v = max(0.0, min(1000.0, float(value_0_1000)))
    return (1.0 - (v / 1000.0)) * stroke_m


def gripper_position_m_to_value(position_m: float, gripper_type: str) -> float:
    stroke_m = stroke_m_from_type(gripper_type)
    m = max(0.0, min(stroke_m, float(position_m)))
    return max(0.0, min(1000.0, 1000.0 - ((m / stroke_m if stroke_m > 1e-9 else 0.0) * 1000.0)))


def default_save_root() -> str:
    # Keep data out of the package directory; follow ROS convention (~/.ros).
    return os.path.join(os.path.expanduser("~"), ".ros", "alicia_d_drag_teaching", "example_motions")


def list_available_motions(root_dir: str) -> List[str]:
    if not os.path.exists(root_dir):
        return []
    motions = []
    for item in os.listdir(root_dir):
        motion_dir = os.path.join(root_dir, item)
        if os.path.isdir(motion_dir) and os.path.exists(os.path.join(motion_dir, "joint_traj.json")):
            motions.append(item)
    return sorted(motions)


@dataclass
class LatestState:
    stamp: float = 0.0
    joint_map: Optional[Dict[str, float]] = None


class DragTeachingNode:
    def __init__(self, args: argparse.Namespace):
        self.args = args
        rospy.init_node("drag_teaching", anonymous=True)

        self.gripper_type = rospy.get_param("~gripper_type", args.gripper_type)
        self.save_root = rospy.get_param("~save_root", args.save_root)

        self.demo_pub = rospy.Publisher("/demonstration", Bool, queue_size=10)
        self.cmd_pub = rospy.Publisher("/joint_commands", JointState, queue_size=10)
        # Driver runtime knob: publish deg/s to update driver default speed.
        # Driver subscribes to /default_speed_deg_s (std_msgs/Float64).
        self.default_speed_pub = rospy.Publisher("/default_speed_deg_s", Float64, queue_size=1, latch=True)
        self.state = LatestState()

        self._sub = rospy.Subscriber("/joint_states", JointState, self._on_joint_state, queue_size=10)

    def _on_joint_state(self, msg: JointState) -> None:
        joint_map: Dict[str, float] = {}
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_map[name] = msg.position[i]
        self.state = LatestState(stamp=time.time(), joint_map=joint_map)

    def _wait_for_state(self, timeout_s: float = 3.0) -> bool:
        start = time.time()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and (time.time() - start) < timeout_s:
            if self.state.joint_map:
                return True
            rate.sleep()
        return False

    def _current_arm_q(self) -> Optional[List[float]]:
        if not self.state.joint_map:
            return None
        return [float(self.state.joint_map.get(j, 0.0)) for j in ARM_JOINTS]

    @staticmethod
    def _deg_s_to_rad_s(deg_s: float) -> float:
        return float(deg_s) * 3.141592653589793 / 180.0

    @staticmethod
    def _velocity_from_two_points(
        q: List[float],
        q_next: List[float],
        dt_s: float,
        speed_factor: float,
    ) -> List[float]:
        """Compute per-joint velocity in rad/s from two joint position vectors."""
        dt = max(1e-3, float(dt_s))
        sf = max(1e-6, float(speed_factor))
        return [(float(q_next[i]) - float(q[i])) / dt * sf for i in range(6)]

    @staticmethod
    def _cap_velocity_rad_s(vel_rad_s: List[float], cap_rad_s: float) -> List[float]:
        """Scale velocity vector so that max(abs(v)) <= cap_rad_s (if cap_rad_s > 0)."""
        cap = float(cap_rad_s)
        if cap <= 1e-9:
            return vel_rad_s
        max_abs = max(abs(v) for v in vel_rad_s) if vel_rad_s else 0.0
        if max_abs <= cap or max_abs <= 1e-12:
            return vel_rad_s
        s = cap / max_abs
        return [v * s for v in vel_rad_s]

    def torque_off(self) -> None:
        for _ in range(3):
            self.demo_pub.publish(Bool(data=True))
            rospy.sleep(0.2)

    def torque_on(self) -> None:
        for _ in range(3):
            self.demo_pub.publish(Bool(data=False))
            rospy.sleep(0.2)

    def _get_current_point(self, start_time: float) -> Optional[Dict]:
        if not self.state.joint_map:
            return None
        q = [float(self.state.joint_map.get(j, 0.0)) for j in ARM_JOINTS]
        grip_pos_m = float(self.state.joint_map.get(GRIPPER_JOINT, 0.0))
        grip_raw = gripper_position_m_to_value(grip_pos_m, self.gripper_type)
        return {"t": time.time() - start_time, "q": q, "grip": grip_raw}

    def record_manual(self) -> List[Dict]:
        print("\n=== Manual keypoint mode ===")
        print("Torque will be turned OFF. Move the arm, press Enter to record a point; type 'q' + Enter to finish.")
        input("Press Enter to start...")
        if not self._wait_for_state():
            print("[ERROR] No /joint_states received. Is the driver running?")
            return []

        self.torque_off()
        print("[SAFETY] Torque OFF. You can now move the robot by hand.")

        start = time.time()
        waypoints: List[Dict] = []
        try:
            while not rospy.is_shutdown():
                cmd = input(f"\nMove to pose and press Enter to record point #{len(waypoints)+1} (or 'q' to finish): ").strip()
                if cmd.lower() == "q":
                    break
                pt = self._get_current_point(start)
                if pt:
                    waypoints.append(pt)
                    print(f"[REC] {len(waypoints)}: q={[round(x,3) for x in pt['q']]}, grip={pt['grip']:.0f}")
        finally:
            self.torque_on()
            print("[SAFETY] Torque ON.")
        return waypoints

    def record_auto(self) -> List[Dict]:
        print("\n=== Auto trajectory mode ===")
        print("Torque will be turned OFF. Move the arm; the system will sample /joint_states and record a trajectory.")
        input("Press Enter to start...")
        if not self._wait_for_state():
            print("[ERROR] No /joint_states received. Is the driver running?")
            return []

        self.torque_off()
        print("[SAFETY] Torque OFF. You can now move the robot by hand.")

        input("Start moving the arm, then press Enter to BEGIN recording...")

        duration_s = float(self.args.duration) if self.args.duration and self.args.duration > 0 else 0.0
        sample_hz = float(self.args.sample_hz)

        start = time.time()
        traj: List[Dict] = []
        recording = threading.Event()
        recording.set()
        stop_requested = threading.Event()

        def record_loop() -> None:
            rate = rospy.Rate(sample_hz if sample_hz > 1e-6 else 100.0)
            while not rospy.is_shutdown() and recording.is_set():
                if duration_s > 0 and (time.time() - start) >= duration_s:
                    break
                pt = self._get_current_point(start)
                if pt:
                    traj.append(pt)
                rate.sleep()
            recording.clear()

        def stop_on_enter() -> None:
            try:
                input("Press Enter to STOP recording...")
                stop_requested.set()
                recording.clear()
            except (EOFError, KeyboardInterrupt):
                stop_requested.set()
                recording.clear()

        thread = threading.Thread(target=record_loop, daemon=True)
        thread.start()

        # Add SDK-style logs (English)
        print(f"[RECORDING] Rate: {sample_hz} Hz...")

        try:
            if sys.stdin.isatty():
                stopper = threading.Thread(target=stop_on_enter, daemon=True)
                stopper.start()

            if duration_s > 0:
                print(f"[RECORDING] Duration: {duration_s:.1f}s (will auto-stop; you can also press Enter).")
                while not rospy.is_shutdown() and recording.is_set():
                    time.sleep(0.05)
                    if stop_requested.is_set():
                        break
            else:
                # If stdin isn't interactive, user can Ctrl+C to stop.
                while not rospy.is_shutdown() and recording.is_set():
                    time.sleep(0.05)
        except KeyboardInterrupt:
            recording.clear()
        finally:
            recording.clear()
            thread.join(timeout=1.0)
            self.torque_on()
            print("[SAFETY] Torque ON.")

        print(f"[DONE] Recorded {len(traj)} points.")
        return traj

    def save_motion(self, motion: str, data: List[Dict]) -> Optional[str]:
        if not data:
            return None
        root = self.save_root
        os.makedirs(root, exist_ok=True)
        motion_dir = os.path.join(root, motion)
        os.makedirs(motion_dir, exist_ok=True)

        traj_path = os.path.join(motion_dir, "joint_traj.json")
        with open(traj_path, "w", encoding="utf-8") as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        meta = {
            "motion": motion,
            "mode": self.args.mode,
            "created_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "sample_hz": self.args.sample_hz,
            "count": len(data),
            "gripper_type": self.gripper_type,
        }
        meta_path = os.path.join(motion_dir, "meta.json")
        with open(meta_path, "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=2)

        print(f"[SAVE] {traj_path}")
        return motion_dir

    def load_motion(self, motion: str) -> Optional[List[Dict]]:
        motion_dir = os.path.join(self.save_root, motion)
        traj_path = os.path.join(motion_dir, "joint_traj.json")
        if not os.path.exists(traj_path):
            print(f"[ERROR] Trajectory file not found: {traj_path}")
            return None
        with open(traj_path, "r", encoding="utf-8") as f:
            return json.load(f)

    def replay(self, data: List[Dict], speed_factor: float) -> None:
        if not data:
            return
        print(f"\n=== Replay trajectory ({len(data)} points) ===")
        if not self._wait_for_state():
            print("[WARN] No /joint_states received; will still attempt replay.")

        # Replay strategy (aligned with SDK expectations):
        # - Always publish each recorded waypoint in order
        # - Preserve recorded timing when replay_speed_deg_s <= 0 (sleep based on recorded dt)
        # - When replay_speed_deg_s > 0, publish a velocity cap (rad/s) and still yield time between points
        #   so the driver/controller can execute commands deterministically.

        replay_speed_deg_s = float(getattr(self.args, "replay_speed_deg_s", 0.0))

        if replay_speed_deg_s > 1e-6:
            # NOTE:
            # - Driver param `default_speed_deg_s` is in deg/s (see alicia_d_driver_node.cpp).
            # - But ROS JointState.velocity is **rad/s** (ROS convention).
            # We publish rad/s here; the driver converts rad/s -> deg/s internally.
            print(f"[REPLAY] Speed cap: {replay_speed_deg_s:.2f} deg/s (published as rad/s in JointState.velocity)")
        else:
            print("[REPLAY] Timing: derived from recorded timestamps (t).")

        pts = data
        n = len(pts)
        last_progress = -1

        for i in range(n):
            if rospy.is_shutdown():
                break

            pt = pts[i]
            next_pt = pts[i + 1] if i + 1 < len(pts) else None

            q = [float(x) for x in pt.get("q", [0.0] * 6)[:6]]
            vel = [0.0] * 6  # rad/s (ROS JointState.velocity unit)
            dt_sleep = 0.0
            if next_pt is not None:
                qn = [float(x) for x in next_pt.get("q", q)[:6]]
                dt = float(next_pt.get("t", 0.0)) - float(pt.get("t", 0.0))
                dt_sleep = max(0.0, dt) / max(1e-6, float(speed_factor))

                # Only publish velocity when user wants an explicit speed cap.
                if replay_speed_deg_s > 1e-6:
                    vel = self._velocity_from_two_points(q, qn, dt, speed_factor)
                    # Convert deg/s (driver unit) -> rad/s (JointState.velocity unit)
                    vel = self._cap_velocity_rad_s(vel, self._deg_s_to_rad_s(replay_speed_deg_s))
                else:
                    vel = []  # no velocity field => driver uses its default_speed_deg_s

            self._publish_point(pt, velocity_rad_s=(vel if vel else None))

            progress = int((i + 1) * 100 / max(1, n))
            if progress != last_progress and (progress % 10 == 0 or i == len(pts) - 1):
                last_progress = progress
                print(f"[REPLAY] Progress: {progress}% ({i+1}/{n})")

            # Yield time between points to preserve recorded timing (SDK-like "replay what you recorded").
            # Even in speed-cap mode, a small dt helps prevent flooding the driver with commands.
            if next_pt is not None:
                if replay_speed_deg_s > 1e-6:
                    # If timestamps are missing/degenerate, fall back to a gentle default.
                    if dt_sleep <= 1e-6:
                        dt_sleep = 1.0 / max(1.0, float(getattr(self.args, "sample_hz", 100.0)))
                if dt_sleep > 1e-6:
                    time.sleep(dt_sleep)

        print("[REPLAY] Done.")

    def _publish_point(self, pt: Dict, velocity_rad_s: Optional[List[float]] = None) -> None:
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name = ARM_JOINTS + [GRIPPER_JOINT]
        q = pt.get("q", [0.0] * 6)
        grip_raw = float(pt.get("grip", 0.0))
        # /joint_commands expects raw gripper 0..1000 (driver_node interprets it as raw)
        msg.position = [float(x) for x in q[:6]] + [max(0.0, min(1000.0, grip_raw))]
        if velocity_rad_s is not None:
            # Driver uses max abs velocity across joints to derive a single speed (deg/s).
            msg.velocity = [float(v) for v in velocity_rad_s[:6]] + [0.0]
        self.cmd_pub.publish(msg)

    def run(self) -> None:
        if self.args.list_motions:
            motions = list_available_motions(self.save_root)
            print("=== Available motions ===")
            for m in motions:
                print(f"- {m}")
            return

        # Optionally set driver default speed (deg/s) once at start.
        driver_default_speed_deg_s = float(getattr(self.args, "driver_default_speed_deg_s", 0.0))
        if driver_default_speed_deg_s > 1e-6:
            msg = Float64(data=driver_default_speed_deg_s)
            for _ in range(3):
                self.default_speed_pub.publish(msg)
                rospy.sleep(0.05)
            print(f"[INFO] Published /default_speed_deg_s = {driver_default_speed_deg_s:.2f} deg/s")

        if self.args.mode in ("manual", "auto") and not self.args.save_motion:
            print("[ERROR] Recording modes require --save-motion")
            return
        if self.args.mode == "replay_only" and not self.args.save_motion:
            print("[ERROR] Replay mode requires --save-motion")
            return

        if self.args.mode == "manual":
            data = self.record_manual()
            if data:
                self.save_motion(self.args.save_motion, data)
                if self.args.replay:
                    self.replay(data, self.args.speed_factor)
        elif self.args.mode == "auto":
            data = self.record_auto()
            if data:
                self.save_motion(self.args.save_motion, data)
                if self.args.replay:
                    self.replay(data, self.args.speed_factor)
        else:  # replay_only
            data = self.load_motion(self.args.save_motion)
            if data:
                self.replay(data, self.args.speed_factor)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Alicia-D Drag Teaching (ROS1)")
    p.add_argument("--mode", choices=["manual", "auto", "replay_only"], default="replay_only")
    p.add_argument("--save-motion", dest="save_motion", default="")
    p.add_argument("--list-motions", action="store_true")
    p.add_argument("--sample-hz", type=float, default=100.0)
    p.add_argument("--speed-factor", type=float, default=1.0)
    p.add_argument("--duration", type=float, default=0.0, help="auto模式录制时长(秒)，0表示手动结束")
    p.add_argument("--replay", action="store_true", help="录制后立即回放")
    p.add_argument(
        "--replay-speed-deg-s",
        type=float,
        default=0.0,
        help=(
            "cap replay speed in deg/s (driver uses deg/s). "
            "We publish JointState.velocity in rad/s (ROS standard) and the driver converts internally. "
            "0 = derive speed from recorded timing."
        ),
    )
    p.add_argument(
        "--driver-default-speed-deg-s",
        type=float,
        default=0.0,
        help="If >0, publish std_msgs/Float64 to /default_speed_deg_s at startup to update driver default speed (deg/s).",
    )
    p.add_argument("--save-root", default=default_save_root())
    p.add_argument("--gripper-type", default="100mm", choices=["50mm", "100mm"])
    # roslaunch will append ROS remapping args like "__name:=node" and "__log:=...".
    # Filter them out so argparse doesn't error out.
    if argv is None:
        argv = sys.argv[1:]
    filtered: List[str] = []
    for a in argv:
        if a.startswith("__") and ":=" in a:
            continue
        if ":=" in a:
            # Generic ROS remap argument, e.g. "/foo:=/bar"
            continue
        filtered.append(a)
    return p.parse_args(filtered)


def main(argv: Optional[List[str]] = None) -> None:
    args = parse_args(argv)
    node = DragTeachingNode(args)
    node.run()


if __name__ == "__main__":
    main()


