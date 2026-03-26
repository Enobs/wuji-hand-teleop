"""
Franka FR3 Dual-Arm Output Node

Subscribes to /joint_states from franka_ik_node and sends joint commands
to two real FR3 robots via franky.

Each arm connects to a separate Franka controller (different IP).
Uses franky's built-in Ruckig trajectory smoothing.

Parameters:
  left_ip          IP of left arm controller  (default "192.168.1.1")
  right_ip         IP of right arm controller (default "192.168.2.1")
  enable_left      Enable left arm  (default true)
  enable_right     Enable right arm (default true)
  dynamic_rel      Ruckig dynamic_rel factor 0.0-1.0 (default 0.2, slower=safer)
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

try:
    from franky import (
        Robot,
        JointWaypointMotion,
        JointWaypoint,
        ReferenceType,
    )
    HAS_FRANKY = True
except ImportError:
    HAS_FRANKY = False


LEFT_JOINTS = [f"left_fr3_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"right_fr3_joint{i}" for i in range(1, 8)]

# FR3 home pose
FR3_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


class FrankaOutputNode(Node):

    def __init__(self):
        super().__init__("franka_output_node")

        self.declare_parameter("left_ip", "192.168.1.1")
        self.declare_parameter("right_ip", "192.168.2.1")
        self.declare_parameter("enable_left", True)
        self.declare_parameter("enable_right", True)
        self.declare_parameter("dynamic_rel", 0.2)

        left_ip = self.get_parameter("left_ip").value
        right_ip = self.get_parameter("right_ip").value
        self._enable_left = self.get_parameter("enable_left").value
        self._enable_right = self.get_parameter("enable_right").value
        dynamic_rel = self.get_parameter("dynamic_rel").value

        if not HAS_FRANKY:
            self.get_logger().fatal("franky not installed: pip install franky-panda")
            return

        # Connect to robots
        self._left_robot = None
        self._right_robot = None

        if self._enable_left:
            try:
                self._left_robot = Robot(left_ip)
                self._left_robot.relative_dynamics_factor = dynamic_rel
                self._left_robot.recover_from_errors()
                self.get_logger().info(f"LEFT  arm connected: {left_ip}")
            except Exception as e:
                self.get_logger().error(f"LEFT  arm failed to connect ({left_ip}): {e}")

        if self._enable_right:
            try:
                self._right_robot = Robot(right_ip)
                self._right_robot.relative_dynamics_factor = dynamic_rel
                self._right_robot.recover_from_errors()
                self.get_logger().info(f"RIGHT arm connected: {right_ip}")
            except Exception as e:
                self.get_logger().error(f"RIGHT arm failed to connect ({right_ip}): {e}")

        # State
        self._left_q = np.array(FR3_HOME)
        self._right_q = np.array(FR3_HOME)
        self._collision = False
        self._enabled = False  # Wait for first valid joint_states

        # Subscriptions
        self.create_subscription(
            JointState, "/joint_states", self._joint_states_cb, 10)
        self.create_subscription(
            Bool, "/collision_state", self._collision_cb, 10)

        # Send commands at 50Hz (franky handles interpolation internally)
        self.create_timer(1.0 / 50.0, self._send_loop)

        self.get_logger().info(
            f"FrankaOutputNode ready — "
            f"left={'ON' if self._left_robot else 'OFF'}, "
            f"right={'ON' if self._right_robot else 'OFF'}, "
            f"dynamic_rel={dynamic_rel}")

    def _joint_states_cb(self, msg: JointState):
        """Extract left/right joint angles from combined joint_states."""
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

    def _send_loop(self):
        if not self._enabled:
            return

        # Don't send new commands during collision (IK is bisecting)
        if self._collision:
            return

        if self._left_robot is not None:
            try:
                motion = JointWaypointMotion([
                    JointWaypoint(self._left_q.tolist())
                ])
                self._left_robot.move(motion, async_=True)
            except Exception as e:
                self.get_logger().warn(
                    f"LEFT move failed: {e}", throttle_duration_sec=2.0)

        if self._right_robot is not None:
            try:
                motion = JointWaypointMotion([
                    JointWaypoint(self._right_q.tolist())
                ])
                self._right_robot.move(motion, async_=True)
            except Exception as e:
                self.get_logger().warn(
                    f"RIGHT move failed: {e}", throttle_duration_sec=2.0)

    def destroy_node(self):
        # Stop robots gracefully
        self.get_logger().info("Stopping robots...")
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
