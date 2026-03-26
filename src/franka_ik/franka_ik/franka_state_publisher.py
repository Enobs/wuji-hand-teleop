"""
Franka FR3 Dual-Arm State Publisher

Reads real joint states from two FR3 robots via franky and publishes
to /joint_states so RViz shows the actual robot pose.

Can run standalone (real robot → RViz) or alongside franka_ik_node
(IK publishes commands, this node publishes real feedback).

Parameters:
  left_ip          IP of left arm  (default "192.168.1.1")
  right_ip         IP of right arm (default "192.168.2.1")
  enable_left      Enable left arm  (default true)
  enable_right     Enable right arm (default true)
  publish_rate     Hz (default 100.0)
  topic            Output topic (default "/joint_states_real")
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from franky import Robot
    HAS_FRANKY = True
except ImportError:
    HAS_FRANKY = False

LEFT_JOINTS = [f"left_fr3_joint{i}" for i in range(1, 8)]
RIGHT_JOINTS = [f"right_fr3_joint{i}" for i in range(1, 8)]
FR3_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


class FrankaStatePublisher(Node):

    def __init__(self):
        super().__init__("franka_state_publisher")

        self.declare_parameter("left_ip", "192.168.1.1")
        self.declare_parameter("right_ip", "192.168.2.1")
        self.declare_parameter("enable_left", True)
        self.declare_parameter("enable_right", True)
        self.declare_parameter("publish_rate", 100.0)
        self.declare_parameter("topic", "/joint_states_real")

        left_ip = self.get_parameter("left_ip").value
        right_ip = self.get_parameter("right_ip").value
        enable_left = self.get_parameter("enable_left").value
        enable_right = self.get_parameter("enable_right").value
        rate_hz = self.get_parameter("publish_rate").value
        topic = self.get_parameter("topic").value

        if not HAS_FRANKY:
            self.get_logger().fatal("franky not installed: pip install franky-panda")
            return

        # Connect
        self._left_robot = None
        self._right_robot = None

        if enable_left:
            try:
                self._left_robot = Robot(left_ip)
                self.get_logger().info(f"LEFT  arm connected: {left_ip}")
            except Exception as e:
                self.get_logger().error(f"LEFT  arm failed ({left_ip}): {e}")

        if enable_right:
            try:
                self._right_robot = Robot(right_ip)
                self.get_logger().info(f"RIGHT arm connected: {right_ip}")
            except Exception as e:
                self.get_logger().error(f"RIGHT arm failed ({right_ip}): {e}")

        self._pub = self.create_publisher(JointState, topic, 10)
        self.create_timer(1.0 / rate_hz, self._loop)
        self.get_logger().info(
            f"Publishing real joint states to {topic} at {rate_hz}Hz")

    def _loop(self):
        left_q = list(self._left_robot.current_joint_position) \
            if self._left_robot else FR3_HOME
        right_q = list(self._right_robot.current_joint_position) \
            if self._right_robot else FR3_HOME

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = LEFT_JOINTS + RIGHT_JOINTS
        msg.position = left_q + right_q
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrankaStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
