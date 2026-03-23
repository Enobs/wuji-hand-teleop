"""ROS2 node that streams Manus glove hand data and publishes ROS topics."""

from __future__ import annotations

import argparse
import sys
from dataclasses import dataclass, fields
from pathlib import Path
from typing import Any, Dict, Optional

import numpy as np

from ament_index_python.packages import get_package_share_directory

try:
    import yaml
except ImportError as exc:
    raise ImportError("PyYAML is required to load Manus input configuration files.") from exc

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, qos_profile_sensor_data
    from std_msgs.msg import Float32MultiArray
    from rclpy.utilities import remove_ros_args
except ImportError as exc:
    raise ImportError("This module requires ROS 2 Python packages (rclpy, std_msgs).") from exc

from manus_ros2_msgs.msg import ManusGlove


# Mapping from MediaPipe index to Manus node_id
# MediaPipe: 0=WRIST, 1-4=THUMB, 5-8=INDEX, 9-12=MIDDLE, 13-16=RING, 17-20=PINKY
#
# Quantum Metaglove node layout (25 nodes):
#   0=Hand(wrist), 1-4=Thumb(MCP,PIP,DIP,TIP), 5-9=Index(MCP,PIP,IP,DIP,TIP),
#   10-14=Middle(MCP,PIP,IP,DIP,TIP), 15-19=Ring(MCP,PIP,IP,DIP,TIP),
#   20-24=Pinky(MCP,PIP,IP,DIP,TIP)
#
# MediaPipe has 4 joints per finger (no IP), so we skip the IP joint:
#   Thumb: MCP(1), PIP(2), DIP(3), TIP(4)
#   Index: MCP(5), PIP(6), DIP(8), TIP(9)  -- skip IP(7)
#   Middle: MCP(10), PIP(11), DIP(13), TIP(14) -- skip IP(12)
#   Ring: MCP(15), PIP(16), DIP(18), TIP(19) -- skip IP(17)
#   Pinky: MCP(20), PIP(21), DIP(23), TIP(24) -- skip IP(22)
MEDIAPIPE_TO_MANUS = (
    0,                    # WRIST (Hand node)
    1, 2, 3, 4,          # THUMB: MCP, PIP, DIP, TIP
    5, 6, 8, 9,          # INDEX: MCP, PIP, DIP, TIP (skip IP=7)
    10, 11, 13, 14,      # MIDDLE: MCP, PIP, DIP, TIP (skip IP=12)
    15, 16, 18, 19,      # RING: MCP, PIP, DIP, TIP (skip IP=17)
    20, 21, 23, 24,      # PINKY: MCP, PIP, DIP, TIP (skip IP=22)
)


@dataclass
class ManusInputConfig:
    """Configuration for the Manus input streaming node."""

    config_path: Optional[str] = None

    # Publishing behaviour
    publish_rate_hz: float = 50.0
    publish_hand_topic: str = "/hand_input"
    include_right_hand: bool = True
    include_left_hand: bool = True

    # Glove IDs
    left_glove_id: int = 0
    right_glove_id: int = 1

    @classmethod
    def from_file(cls, path: str | Path) -> "ManusInputConfig":
        """Load configuration from a YAML file."""
        cfg_path = Path(path).expanduser().resolve()
        if not cfg_path.exists():
            raise FileNotFoundError(f"Config file not found: {cfg_path}")

        with cfg_path.open("r", encoding="utf-8") as handle:
            raw = yaml.safe_load(handle) or {}

        if not isinstance(raw, dict):
            raise ValueError("ManusInputConfig expects a mapping at the root of the YAML file.")

        valid_fields = {field.name for field in fields(cls)}
        data: Dict[str, Any] = {"config_path": str(cfg_path)}
        ignored_keys: list[str] = []

        for key, value in raw.items():
            if key in valid_fields:
                data[key] = value
            else:
                ignored_keys.append(key)

        if ignored_keys:
            print(f"[ManusInputConfig] Ignoring unknown keys: {', '.join(sorted(ignored_keys))}")

        return cls(**data)


class ManusInputNode(Node):
    """ROS2 node that reads Manus glove data and publishes ROS topics."""

    def __init__(self, config: ManusInputConfig):
        super().__init__("manus_input")
        self.config = config

        # Storage for latest data from each glove
        self._left_fingers: Optional[np.ndarray] = None
        self._right_fingers: Optional[np.ndarray] = None
        # 新鲜度标志: 仅当 C++ 层有新数据到达时才发布
        # 防止手套断连后无限重发过期数据导致灵巧手抖动
        self._new_data_received: bool = False

        # Configure QoS for real-time performance
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publisher
        self.hand_publisher = self.create_publisher(
            Float32MultiArray, self.config.publish_hand_topic, qos_profile
        )

        # Subscribe to all manus_glove topics, use msg.side to determine left/right
        # Use qos_profile_sensor_data to match C++ SensorDataQoS publisher
        self.create_subscription(ManusGlove, "/manus_glove_0", self._glove_callback, qos_profile_sensor_data)
        self.create_subscription(ManusGlove, "/manus_glove_1", self._glove_callback, qos_profile_sensor_data)
        self.get_logger().info("[Manus Input] Subscribed to /manus_glove_0 and /manus_glove_1")

        # Timer for publishing
        self.timer = self.create_timer(
            1.0 / max(self.config.publish_rate_hz, 1.0), self._publish_latest_frame
        )
        self.get_logger().info(
            f"[Manus Input] Publishing hand data on '{self.config.publish_hand_topic}' "
            f"at {self.config.publish_rate_hz:.1f} Hz."
        )

    def _convert_to_mediapipe(self, msg: ManusGlove) -> np.ndarray:
        """Convert Manus raw nodes to MediaPipe (21, 3) format."""
        # Store Manus node positions by node_id
        manus_positions = {}

        for node in msg.raw_nodes:
            node_id = node.node_id
            pose = node.pose

            # Position with Y-axis flipped (same as manus_data_viz)
            x = pose.position.x
            y = -pose.position.y
            z = pose.position.z

            manus_positions[node_id] = np.array([x, y, z], dtype=np.float32)

        # Debug: log node IDs and chain types once
        if not hasattr(self, '_nodes_logged'):
            self._nodes_logged = True
            node_info = [(n.node_id, n.chain_type, n.joint_type) for n in msg.raw_nodes]
            self.get_logger().info(f"[DEBUG NODES] {node_info}")
            missing = [mid for mid in MEDIAPIPE_TO_MANUS if mid not in manus_positions]
            if missing:
                self.get_logger().warn(f"[DEBUG] Missing node IDs in mapping: {missing}")

        # Convert to MediaPipe (21, 3) format
        mediapipe_pose = np.zeros((21, 3), dtype=np.float32)

        for mp_idx, manus_node_id in enumerate(MEDIAPIPE_TO_MANUS):
            if manus_node_id in manus_positions:
                mediapipe_pose[mp_idx] = manus_positions[manus_node_id]

        return mediapipe_pose

    def _glove_callback(self, msg: ManusGlove) -> None:
        """Callback for glove data, determines left/right based on msg.side."""
        self.get_logger().info(f"[DEBUG] Got glove data! side='{msg.side}', nodes={len(msg.raw_nodes)}", throttle_duration_sec=2.0)
        mediapipe_data = self._convert_to_mediapipe(msg)
        if msg.side.lower() == "left":
            self._left_fingers = mediapipe_data
            self._new_data_received = True
        elif msg.side.lower() == "right":
            self._right_fingers = mediapipe_data
            self._new_data_received = True
        else:
            self.get_logger().warning(f"Unknown side: {msg.side}")

    def _publish_latest_frame(self) -> None:
        """Publish the latest hand data. Skip if no valid data received yet.

        仅当 C++ 层有新数据到达时才发布，防止手套断连后
        无限重发过期数据导致灵巧手抖动。

        Data format: [right_hand (63), left_hand (63)] - MediaPipe 21-point format.
        """
        # 无新数据: 不发布，避免过期数据导致下游持续 retarget
        if not self._new_data_received:
            return

        # 先检查数据完整性，再清除标志 (防止数据丢失)
        if self.config.include_right_hand and self._right_fingers is None:
            return  # 右手数据尚未到达，保留标志等待下次重试
        if self.config.include_left_hand and self._left_fingers is None:
            return  # 左手数据尚未到达，保留标志等待下次重试

        # 数据齐全，清除标志并发布
        self._new_data_received = False

        payloads = []
        if self.config.include_right_hand:
            payloads.append(self._right_fingers.flatten())
        if self.config.include_left_hand:
            payloads.append(self._left_fingers.flatten())

        if payloads:
            hand_msg = Float32MultiArray()
            hand_msg.data = np.concatenate(payloads).astype(np.float32).tolist()
            self.hand_publisher.publish(hand_msg)


def _parse_args(argv: Optional[list[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish Manus glove hand data to ROS2 topics.")
    parser.add_argument(
        "-c",
        "--config",
        default=None,
        help="Path to a Manus input YAML configuration file.",
    )
    return parser.parse_args(argv)


def main(argv: Optional[list[str]] = None) -> None:
    program_name = sys.argv[0] if sys.argv else "manus_input"
    raw_argv = sys.argv if argv is None else [program_name, *argv]
    cli_argv = remove_ros_args(raw_argv)[1:]
    args = _parse_args(cli_argv)

    if args.config:
        config_path = Path(args.config)
    else:
        share_dir = Path(get_package_share_directory("manus_input_py"))
        config_path = share_dir / "config" / "manus_input.yaml"

    config = ManusInputConfig.from_file(str(config_path))

    rclpy.init(args=raw_argv)
    node = ManusInputNode(config)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv[1:])
