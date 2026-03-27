"""
Franka FR3 Dual-Arm Output Node (Velocity Streaming Mode)

Subscribes to /joint_states from franka_ik_node and sends velocity commands
to real FR3 robots via franky's JointVelocityMotion.

Uses proportional control: vel = Kp * (target_q - current_q), clamped to
velocity limits. This avoids trajectory replanning discontinuities that
cause Reflex errors in position waypoint mode.

Parameters:
  left_ip          IP of left arm controller  (default "192.168.1.1")
  right_ip         IP of right arm controller (default "192.168.2.1")
  enable_left      Enable left arm  (default true)
  enable_right     Enable right arm (default true)
  dynamic_rel      Velocity limit factor 0.0-1.0 (default 0.3)
  kp               Proportional gain (default 5.0, higher=faster tracking)
"""
from __future__ import annotations

import time
import threading
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from franky import (
        Robot,
        JointVelocityMotion,
        RealtimeConfig,
    )
    HAS_FRANKY = True
except ImportError:
    HAS_FRANKY = False


LEFT_JOINTS = [f"left_fr3_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"right_fr3_joint{i}" for i in range(1, 8)]

FR3_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]

# FR3 max joint velocities (rad/s) from datasheet
FR3_MAX_VEL = np.array([2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26])


class FrankaOutputNode(Node):

    def __init__(self):
        super().__init__("franka_output_node")

        self.declare_parameter("left_ip", "192.168.1.1")
        self.declare_parameter("right_ip", "192.168.2.1")
        self.declare_parameter("enable_left", True)
        self.declare_parameter("enable_right", True)
        self.declare_parameter("dynamic_rel", 0.3)
        self.declare_parameter("kp", 5.0)

        left_ip = self.get_parameter("left_ip").value
        right_ip = self.get_parameter("right_ip").value
        self._enable_left = self.get_parameter("enable_left").value
        self._enable_right = self.get_parameter("enable_right").value
        self._dynamic_rel = self.get_parameter("dynamic_rel").value
        self._kp = self.get_parameter("kp").value

        if not HAS_FRANKY:
            self.get_logger().fatal("franky not installed: pip install franky-panda")
            return

        self._vel_limit = FR3_MAX_VEL * self._dynamic_rel

        # Connect to robots
        self._left_robot = None
        self._right_robot = None

        if self._enable_left:
            try:
                self._left_robot = Robot(left_ip, realtime_config=RealtimeConfig.Ignore)
                self._left_robot.relative_dynamics_factor = self._dynamic_rel
                self._left_robot.recover_from_errors()
                self.get_logger().info(f"LEFT  arm connected: {left_ip}")
            except Exception as e:
                self.get_logger().error(f"LEFT  arm failed to connect ({left_ip}): {e}")

        if self._enable_right:
            try:
                self._right_robot = Robot(right_ip, realtime_config=RealtimeConfig.Ignore)
                self._right_robot.relative_dynamics_factor = self._dynamic_rel
                self._right_robot.recover_from_errors()
                self.get_logger().info(f"RIGHT arm connected: {right_ip}")
            except Exception as e:
                self.get_logger().error(f"RIGHT arm failed to connect ({right_ip}): {e}")

        # Initialize targets from real robot positions
        self._left_q = np.array(
            list(self._left_robot.current_joint_positions)
            if self._left_robot else FR3_HOME)
        self._right_q = np.array(
            list(self._right_robot.current_joint_positions)
            if self._right_robot else FR3_HOME)

        self._collision = False
        self._enabled = False
        self._running = True

        # Subscriptions
        self.create_subscription(
            JointState, "/joint_states", self._joint_states_cb, 10)
        self.create_subscription(
            Bool, "/collision_state", self._collision_cb, 10)

        # Start velocity streaming threads
        if self._left_robot is not None:
            self._start_vel_stream(self._left_robot, "LEFT")
        if self._right_robot is not None:
            self._start_vel_stream(self._right_robot, "RIGHT")

        self.get_logger().info(
            f"FrankaOutputNode ready (velocity mode) — "
            f"left={'ON' if self._left_robot else 'OFF'}, "
            f"right={'ON' if self._right_robot else 'OFF'}, "
            f"dynamic_rel={self._dynamic_rel}, Kp={self._kp}")

    def _start_vel_stream(self, robot: "Robot", label: str):
        """Velocity streaming loop in a separate thread.

        Sends short-duration velocity motions at ~100Hz.
        Each motion: compute P-controller velocity, clamp, send.
        """
        def loop():
            dt = 0.01  # 100Hz
            while self._running:
                if not self._enabled or self._collision:
                    time.sleep(dt)
                    continue

                try:
                    current_q = np.array(robot.current_joint_positions)
                    target_q = self._left_q if label == "LEFT" else self._right_q

                    # P-controller
                    error = target_q - current_q
                    vel = self._kp * error

                    # Clamp to velocity limits
                    vel = np.clip(vel, -self._vel_limit, self._vel_limit)

                    # Send velocity command (short duration, will be replaced next cycle)
                    motion = JointVelocityMotion(vel)
                    robot.move(motion, asynchronous=True)

                except Exception as e:
                    err = str(e)
                    if "Reflex" in err or "current mode" in err:
                        try:
                            robot.recover_from_errors()
                            self.get_logger().warn(
                                f"{label} Reflex → auto recovered",
                                throttle_duration_sec=1.0)
                        except Exception:
                            pass
                    else:
                        self.get_logger().warn(
                            f"{label} vel failed: {e}",
                            throttle_duration_sec=2.0)

                time.sleep(dt)

        thread = threading.Thread(target=loop, daemon=True)
        thread.start()
        self.get_logger().info(f"{label} velocity stream started (Kp={self._kp})")

    def _joint_states_cb(self, msg: JointState):
        if len(msg.position) < 14:
            return

        name_to_pos = dict(zip(msg.name, msg.position))

        left_q = [name_to_pos.get(j, FR3_HOME[i]) for i, j in enumerate(LEFT_JOINTS)]
        right_q = [name_to_pos.get(j, FR3_HOME[i]) for i, j in enumerate(RIGHT_JOINTS)]

        self._left_q = np.array(left_q)
        self._right_q = np.array(right_q)
        self._enabled = True

    def _collision_cb(self, msg: Bool):
        self._collision = msg.data

    def destroy_node(self):
        self.get_logger().info("Stopping robots...")
        self._running = False
        time.sleep(0.05)
        for robot in [self._left_robot, self._right_robot]:
            if robot is not None:
                try:
                    robot.stop()
                except Exception:
                    pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = FrankaOutputNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
