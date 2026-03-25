#!/usr/bin/env python3
"""
Test Franka IK by publishing simple target poses.

Usage:
  # Terminal 1: launch without pico input
  ROS_DOMAIN_ID=42 ros2 launch franka_ik franka_pico_viz.launch.py data_source_type:=recorded

  # Terminal 2: run this script
  ROS_DOMAIN_ID=42 python3 src/franka_ik/test/test_ik_targets.py

Modes:
  1. Static: both arms at home EE position
  2. Sine: right arm moves in X (forward/back)
  3. Circle: right arm traces a circle in XZ plane
"""
import sys
import time
import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


# FR3 approximate home EE positions in world frame
# (left base at Y=+0.3, right base at Y=-0.3)
LEFT_HOME  = [0.307, 0.3, 0.590]
RIGHT_HOME = [0.307, -0.3, 0.590]

# Home EE orientation (from FK at home joints)
HOME_QUAT = [0.9240, -0.3825, 0.0, 0.0]  # [qx, qy, qz, qw]


class IKTestNode(Node):
    def __init__(self, mode="static"):
        super().__init__("ik_test_node")
        self._left_pub = self.create_publisher(PoseStamped, "/left_arm_target_pose", 10)
        self._right_pub = self.create_publisher(PoseStamped, "/right_arm_target_pose", 10)
        self._mode = mode
        self._t0 = time.time()
        self.create_timer(1.0 / 50.0, self._loop)
        self.get_logger().info(f"IK test mode: {mode}")

    def _make_msg(self, pos, quat=HOME_QUAT):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = pos[0]
        msg.pose.position.y = pos[1]
        msg.pose.position.z = pos[2]
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]
        return msg

    def _loop(self):
        t = time.time() - self._t0

        # Left arm: always at home
        self._left_pub.publish(self._make_msg(LEFT_HOME))

        if self._mode == "static":
            self._right_pub.publish(self._make_msg(RIGHT_HOME))

        elif self._mode == "sine":
            # Move right arm forward/back +-10cm
            x = RIGHT_HOME[0] + 0.1 * math.sin(t * 0.5)
            self._right_pub.publish(self._make_msg([x, RIGHT_HOME[1], RIGHT_HOME[2]]))

        elif self._mode == "circle":
            # Right arm traces circle in XZ plane, radius 8cm
            r = 0.08
            x = RIGHT_HOME[0] + r * math.sin(t * 0.5)
            z = RIGHT_HOME[2] + r * math.cos(t * 0.5)
            self._right_pub.publish(self._make_msg([x, RIGHT_HOME[1], z]))

        elif self._mode == "axes":
            # Cycle through X, Y, Z movement (5s each)
            phase = int(t / 5.0) % 3
            offset = 0.1 * math.sin(t * 0.5)
            pos = list(RIGHT_HOME)
            axis_name = ["X(forward)", "Y(left)", "Z(up)"][phase]
            pos[phase] += offset
            self._right_pub.publish(self._make_msg(pos))
            if int(t * 50) % 250 == 0:
                self.get_logger().info(f"Axis: {axis_name}, offset={offset:.3f}")

        if int(t * 50) % 500 == 0:
            self.get_logger().info(f"t={t:.1f}s, mode={self._mode}")


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else "static"
    rclpy.init()
    node = IKTestNode(mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
