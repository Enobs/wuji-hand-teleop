"""
Franka PICO Input Node

Reads PICO tracker data and publishes world-frame PoseStamped targets for
the franka_ik_node. No dependency on tianji_world_output.

Coordinate transform (PICO → Franka world):
  Robot_X = -PICO_Z   (forward)
  Robot_Y = -PICO_X   (left)
  Robot_Z =  PICO_Y   (up)

Control flow:
  1. Data source initializes (recorded file or live PICO SDK)
  2. After auto_init_delay seconds, record calibration positions
  3. Each cycle: target_world = init_ee_world + R @ (pico_pos - calib_pico_pos)
  4. Orientation: transform PICO tracker quaternion into world frame

Subscribes: (none)
Publishes:
  /left_arm_target_pose   (geometry_msgs/PoseStamped)
  /right_arm_target_pose  (geometry_msgs/PoseStamped)

Parameters:
  data_source_type       "live" | "recorded"
  recorded_file_path     path to .txt recording
  playback_speed         float (default 1.0)
  auto_init_delay        seconds before calibration (default 3.0)
  publish_rate           Hz (default 90.0)
  left_init_pos          [x, y, z] world frame (default [0.3, 0.3, 0.5])
  right_init_pos         [x, y, z] world frame (default [0.3,-0.3, 0.5])
  tracker_serial_left    serial suffix for left wrist  (default "190058")
  tracker_serial_right   serial suffix for right wrist (default "190600")
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

# PICO data-source abstraction (no tianji dependency)
from pico_input.data_source.recorded_data_source import RecordedDataSource
from pico_input.data_source.live_data_source import LiveDataSource
from pico_input.data_source.base import DataSource, TrackerData


# PICO → Franka-world rotation matrix
# Robot_X = -PICO_Z, Robot_Y = -PICO_X, Robot_Z = PICO_Y
_R_PICO2WORLD = np.array([
    [ 0,  0, -1],
    [-1,  0,  0],
    [ 0,  1,  0],
], dtype=np.float64)


def _transform_pos(pico_pos: np.ndarray) -> np.ndarray:
    """Transform position vector from PICO frame to world frame."""
    return _R_PICO2WORLD @ pico_pos


def _transform_rot(pico_quat: np.ndarray) -> np.ndarray:
    """
    Transform quaternion from PICO frame to world frame.
    Returns quaternion [qx, qy, qz, qw].
    """
    R_tracker = R.from_quat(pico_quat).as_matrix()
    R_world = _R_PICO2WORLD @ R_tracker @ _R_PICO2WORLD.T
    return R.from_matrix(R_world).as_quat()


def _make_pose_msg(pos: np.ndarray, quat: np.ndarray, stamp, frame: str) -> PoseStamped:
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose.position.x = float(pos[0])
    msg.pose.position.y = float(pos[1])
    msg.pose.position.z = float(pos[2])
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


class FrankaPicoInputNode(Node):

    def __init__(self):
        super().__init__("franka_pico_input_node")

        self.declare_parameter("data_source_type",    "live")
        self.declare_parameter("recorded_file_path",  "")
        self.declare_parameter("playback_speed",      1.0)
        self.declare_parameter("auto_init_delay",     3.0)
        self.declare_parameter("publish_rate",        90.0)
        self.declare_parameter("left_init_pos",  [0.3,  0.3, 0.5])
        self.declare_parameter("right_init_pos", [0.3, -0.3, 0.5])
        self.declare_parameter("tracker_serial_left",  "190058")
        self.declare_parameter("tracker_serial_right", "190600")

        src_type     = self.get_parameter("data_source_type").value
        file_path    = self.get_parameter("recorded_file_path").value
        speed        = self.get_parameter("playback_speed").value
        self._delay  = self.get_parameter("auto_init_delay").value
        rate_hz      = self.get_parameter("publish_rate").value
        self._left_init  = np.array(self.get_parameter("left_init_pos").value,  dtype=np.float64)
        self._right_init = np.array(self.get_parameter("right_init_pos").value, dtype=np.float64)
        self._sn_left    = self.get_parameter("tracker_serial_left").value
        self._sn_right   = self.get_parameter("tracker_serial_right").value

        # ── data source ────────────────────────────────────────────────
        self._source: DataSource
        if src_type == "recorded":
            self._source = RecordedDataSource(file_path, playback_speed=speed, loop=True)
        else:
            self._source = LiveDataSource()

        ok = self._source.initialize()
        if not ok:
            self.get_logger().error(
                f"Data source '{src_type}' failed to initialize "
                f"(file='{file_path}'). Node will keep retrying.")
        else:
            self.get_logger().info(f"Data source '{src_type}' ready")

        # ── calibration state ──────────────────────────────────────────
        self._calib_time: Optional[float] = None   # when to calibrate
        self._calib_left_pico:  Optional[np.ndarray] = None
        self._calib_right_pico: Optional[np.ndarray] = None
        self._calib_left_rot:   Optional[np.ndarray] = None  # world-frame quat at calib
        self._calib_right_rot:  Optional[np.ndarray] = None
        self._calibrated = False

        # Start calibration countdown immediately
        self._calib_time = time.monotonic() + self._delay
        self.get_logger().info(
            f"Calibration in {self._delay:.1f}s — hold wrists at natural pose")

        # ── publishers ─────────────────────────────────────────────────
        self._left_pub  = self.create_publisher(PoseStamped, "/left_arm_target_pose",  10)
        self._right_pub = self.create_publisher(PoseStamped, "/right_arm_target_pose", 10)

        self.create_timer(1.0 / rate_hz, self._loop)
        self.get_logger().info(
            f"FrankaPicoInputNode ready — {rate_hz}Hz, source={src_type}, "
            f"left_sn=*{self._sn_left}, right_sn=*{self._sn_right}")

    # ── helpers ────────────────────────────────────────────────────────

    def _find_tracker(self, trackers: list[TrackerData], suffix: str) -> Optional[TrackerData]:
        """Return first tracker whose serial number ends with suffix."""
        for t in trackers:
            if t.serial_number.endswith(suffix) and t.is_valid:
                return t
        return None

    # ── main loop ──────────────────────────────────────────────────────

    def _loop(self):
        if not self._source.is_available():
            return

        trackers = self._source.get_tracker_data()
        if not trackers:
            return

        left_t  = self._find_tracker(trackers, self._sn_left)
        right_t = self._find_tracker(trackers, self._sn_right)

        # ── calibration ────────────────────────────────────────────────
        now = time.monotonic()
        if not self._calibrated and self._calib_time is not None and now >= self._calib_time:
            if left_t is not None and right_t is not None:
                self._calib_left_pico  = left_t.position.copy()
                self._calib_right_pico = right_t.position.copy()
                self._calib_left_rot   = _transform_rot(left_t.orientation)
                self._calib_right_rot  = _transform_rot(right_t.orientation)
                self._calibrated = True
                self.get_logger().info(
                    f"Calibrated — left_pico={self._calib_left_pico}, "
                    f"right_pico={self._calib_right_pico}")
            else:
                # Retry in 0.5s if trackers not yet visible
                self._calib_time = now + 0.5
                self.get_logger().warn(
                    "Calibration: waiting for both wrist trackers…",
                    throttle_duration_sec=2.0)
            return

        if not self._calibrated:
            return

        stamp = self.get_clock().now().to_msg()

        if left_t is not None:
            delta_world = _transform_pos(left_t.position - self._calib_left_pico)
            pos_world   = self._left_init + delta_world
            quat_world  = _transform_rot(left_t.orientation)
            self._left_pub.publish(
                _make_pose_msg(pos_world, quat_world, stamp, "world"))

        if right_t is not None:
            delta_world = _transform_pos(right_t.position - self._calib_right_pico)
            pos_world   = self._right_init + delta_world
            quat_world  = _transform_rot(right_t.orientation)
            self._right_pub.publish(
                _make_pose_msg(pos_world, quat_world, stamp, "world"))

    def destroy_node(self):
        self._source.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaPicoInputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
