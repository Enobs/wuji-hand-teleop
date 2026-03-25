"""
Franka PICO Input Node (WebXR + recorded + live)

Calibration:
  - Auto-calibrate after startup delay
  - Press BOTH triggers simultaneously to re-calibrate anytime
  - Press grip (button 1) to toggle pause/resume

Control:
  target_world = init_ee + scale * R_pico2world @ (pico_current - pico_calib)

WebXR coordinate convention (OpenGL):
  X=right, Y=up, Z=backward

Franka world:
  X=forward, Y=left, Z=up

Transform:
  Franka_X = -WebXR_Z
  Franka_Y = -WebXR_X
  Franka_Z =  WebXR_Y
"""
from __future__ import annotations

import time
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from pico_input.data_source.recorded_data_source import RecordedDataSource
from pico_input.data_source.live_data_source import LiveDataSource
from pico_input.data_source.base import TrackerData
from franka_ik.webxr_data_source import WebXRDataSource

# PICO WebXR → Franka world rotation
# PICO: X=forward, Y=right, Z=up
# Franka: X=forward, Y=left, Z=up
_R_PICO2WORLD = np.array([
    [ 1,  0,  0],   # Franka_X = +PICO_X (forward)
    [ 0,  1,  0],   # Franka_Y = +PICO_Y
    [ 0,  0,  1],   # Franka_Z = +PICO_Z (up)
], dtype=np.float64)


def _transform_pos(pico_delta: np.ndarray) -> np.ndarray:
    return _R_PICO2WORLD @ pico_delta


def _transform_rot(pico_quat: np.ndarray) -> np.ndarray:
    R_tracker = R.from_quat(pico_quat).as_matrix()
    R_world = _R_PICO2WORLD @ R_tracker @ _R_PICO2WORLD.T
    return R.from_matrix(R_world).as_quat()


def _make_pose_msg(pos, quat, stamp, frame="world"):
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

        # Parameters
        self.declare_parameter("data_source_type", "live")
        self.declare_parameter("recorded_file_path", "")
        self.declare_parameter("playback_speed", 1.0)
        self.declare_parameter("auto_init_delay", 3.0)
        self.declare_parameter("publish_rate", 90.0)
        self.declare_parameter("left_init_pos",  [0.307,  0.3, 0.590])
        self.declare_parameter("right_init_pos", [0.307, -0.3, 0.590])
        self.declare_parameter("scale", 1.0)
        self.declare_parameter("tracker_serial_left", "190058")
        self.declare_parameter("tracker_serial_right", "190600")

        src_type  = self.get_parameter("data_source_type").value
        file_path = self.get_parameter("recorded_file_path").value
        speed     = self.get_parameter("playback_speed").value
        self._delay    = self.get_parameter("auto_init_delay").value
        rate_hz        = self.get_parameter("publish_rate").value
        self._left_init  = np.array(self.get_parameter("left_init_pos").value, dtype=np.float64)
        self._right_init = np.array(self.get_parameter("right_init_pos").value, dtype=np.float64)
        self._scale      = self.get_parameter("scale").value
        self._sn_left    = self.get_parameter("tracker_serial_left").value
        self._sn_right   = self.get_parameter("tracker_serial_right").value

        self._src_type = src_type
        if src_type == "webxr":
            self._sn_left = "WEBXR_LEFT"
            self._sn_right = "WEBXR_RIGHT"

        # Data source
        if src_type == "recorded":
            self._source = RecordedDataSource(file_path, playback_speed=speed, loop=True)
        elif src_type == "webxr":
            self._source = WebXRDataSource()
        else:
            self._source = LiveDataSource()

        ok = self._source.initialize()
        self.get_logger().info(
            f"Data source '{src_type}' {'ready' if ok else 'FAILED'}")

        # Home EE orientation (from FK at home joints, qx,qy,qz,qw)
        self._home_quat = np.array([0.9240, -0.3825, 0.0, 0.0])

        # Calibration state
        self._calib_left_pico: Optional[np.ndarray] = None
        self._calib_right_pico: Optional[np.ndarray] = None
        self._calib_left_rot: Optional[R] = None   # pico rotation at calibration
        self._calib_right_rot: Optional[R] = None
        self._calibrated = False
        self._paused = False
        self._calib_time = time.monotonic() + self._delay
        self._prev_both_triggers = False

        # Debug: track axis movement
        self._debug_counter = 0

        # Publishers
        self._left_pub = self.create_publisher(PoseStamped, "/left_arm_target_pose", 10)
        self._right_pub = self.create_publisher(PoseStamped, "/right_arm_target_pose", 10)

        self.create_timer(1.0 / rate_hz, self._loop)
        self.get_logger().info(
            f"Ready — {rate_hz}Hz, scale={self._scale}, "
            f"init_L={self._left_init.tolist()}, init_R={self._right_init.tolist()}")
        self.get_logger().info(
            f"Controls: BOTH TRIGGERS = calibrate, GRIP = pause/resume")

    def _find_tracker(self, trackers, suffix):
        for t in trackers:
            if t.serial_number.endswith(suffix) and t.is_valid:
                return t
        return None

    def _check_buttons(self):
        """Check for trigger/grip presses. Returns (both_triggers_pressed, grip_toggled)."""
        if not isinstance(self._source, WebXRDataSource):
            return False, False

        buttons = self._source.get_buttons()
        lb = buttons.get("left", [])
        rb = buttons.get("right", [])

        # Button 0 = trigger, Button 1 = grip
        left_trigger = len(lb) > 0 and lb[0].get("pressed", False)
        right_trigger = len(rb) > 0 and rb[0].get("pressed", False)
        both_triggers = left_trigger and right_trigger

        # Detect rising edge for trigger
        trigger_edge = both_triggers and not self._prev_both_triggers
        self._prev_both_triggers = both_triggers

        # Grip toggle (either hand)
        left_grip = len(lb) > 1 and lb[1].get("pressed", False)
        right_grip = len(rb) > 1 and rb[1].get("pressed", False)

        return trigger_edge, (left_grip or right_grip)

    def _calibrate(self, left_t, right_t):
        self._calib_left_pico = left_t.position.copy()
        self._calib_right_pico = right_t.position.copy()
        self._calib_left_rot = R.from_quat(left_t.orientation)
        self._calib_right_rot = R.from_quat(right_t.orientation)
        self._calibrated = True
        self.get_logger().info(
            f"CALIBRATED — left_pico={self._calib_left_pico.round(3)}, "
            f"right_pico={self._calib_right_pico.round(3)}")

    def _loop(self):
        if not self._source.is_available():
            return

        trackers = self._source.get_tracker_data()
        if not trackers:
            return

        left_t = self._find_tracker(trackers, self._sn_left)
        right_t = self._find_tracker(trackers, self._sn_right)

        # Button checks
        trigger_pressed, grip_pressed = self._check_buttons()

        if trigger_pressed and left_t and right_t:
            self._calibrate(left_t, right_t)
            return

        if grip_pressed:
            self._paused = not self._paused
            self.get_logger().info(f"{'PAUSED' if self._paused else 'RESUMED'}")
            time.sleep(0.3)  # debounce
            return

        # Auto calibration on startup
        if not self._calibrated:
            now = time.monotonic()
            if self._calib_time and now >= self._calib_time:
                if left_t and right_t:
                    self._calibrate(left_t, right_t)
                else:
                    self._calib_time = now + 0.5
                    self.get_logger().warn(
                        "Waiting for both controllers...",
                        throttle_duration_sec=2.0)
            return

        if self._paused:
            return

        stamp = self.get_clock().now().to_msg()

        home_rot = R.from_quat(self._home_quat)

        if left_t is not None:
            delta_pico = left_t.position - self._calib_left_pico
            delta_world = self._scale * _transform_pos(delta_pico)
            pos_world = self._left_init + delta_world
            # Relative rotation: delta_rot = current @ calib_inv, then apply to home
            delta_rot = R.from_quat(left_t.orientation) * self._calib_left_rot.inv()
            quat_world = (delta_rot * home_rot).as_quat()
            self._left_pub.publish(_make_pose_msg(pos_world, quat_world, stamp))

        if right_t is not None:
            delta_pico = right_t.position - self._calib_right_pico
            delta_world = self._scale * _transform_pos(delta_pico)
            pos_world = self._right_init + delta_world
            delta_rot = R.from_quat(right_t.orientation) * self._calib_right_rot.inv()
            quat_world = (delta_rot * home_rot).as_quat()
            self._right_pub.publish(_make_pose_msg(pos_world, quat_world, stamp))

        # Debug: print delta every ~1s
        self._debug_counter += 1
        if self._debug_counter % 90 == 0 and right_t is not None:
            dp = right_t.position - self._calib_right_pico
            dw = self._scale * _transform_pos(dp)
            self.get_logger().info(
                f"R delta_pico=[{dp[0]:+.3f},{dp[1]:+.3f},{dp[2]:+.3f}] "
                f"delta_world=[{dw[0]:+.3f},{dw[1]:+.3f},{dw[2]:+.3f}] "
                f"target={np.round(self._right_init + dw, 3).tolist()}",
                throttle_duration_sec=1.0)

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
