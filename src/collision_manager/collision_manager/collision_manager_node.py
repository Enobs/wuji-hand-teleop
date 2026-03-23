"""
Collision Manager ROS2 Node

Uses URDF collision meshes (STL) with FCL for accurate real-time
collision detection. Reads link transforms from TF tree.

Usage:
    ros2 run collision_manager collision_manager
"""
from __future__ import annotations

from pathlib import Path
from typing import Dict, List, Optional

import numpy as np
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, ColorRGBA
from std_srvs.srv import SetBool
from geometry_msgs.msg import Vector3, Quaternion, Point
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import TransformException
from scipy.spatial.transform import Rotation as R

from .mesh_collision_checker import MeshCollisionChecker, CollisionResult


# FR3 link names in URDF TF tree
FR3_LINKS = ["fr3_link0", "fr3_link1", "fr3_link2", "fr3_link3",
             "fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7", "fr3_link8"]


class CollisionManagerNode(Node):
    """Mesh-based collision detection node for dual-arm teleoperation."""

    def __init__(self):
        super().__init__("collision_manager")

        # Load config
        config_path = self._find_config()
        self.config = self._load_config(config_path)
        self.get_logger().info(f"Config: {config_path}")

        safety_margin = self.config.get("safety_margin", 0.03)

        # Build exclude pairs
        exclude = self._build_exclude_pairs()

        # Initialize mesh collision checker
        self.checker = MeshCollisionChecker(
            safety_margin=safety_margin,
            exclude_pairs=exclude,
        )

        # Find mesh directory
        self._mesh_dir = self._find_mesh_dir()
        self.get_logger().info(f"Mesh dir: {self._mesh_dir}")

        # Load meshes for both arms
        self._load_arm_meshes("left")
        self._load_arm_meshes("right")

        # Load environment
        self._load_environment()

        # TF
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._base_frame = "world"

        # Publishers
        self.collision_pub = self.create_publisher(Bool, "/collision_state", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/collision_markers", 10)

        # Service
        self._enabled = True
        self.create_service(SetBool, "/collision/enable", self._enable_cb)

        # Timer
        rate = self.config.get("check_rate_hz", 100.0)
        self.create_timer(1.0 / rate, self._check_loop)

        body_count = len(self.checker.bodies)
        env_count = len(self.checker.env_bodies)
        total_faces = sum(b.face_count for b in self.checker.bodies.values())
        self.get_logger().info(
            f"Ready: {body_count} bodies ({total_faces} faces total), "
            f"{env_count} env, {rate}Hz, margin={safety_margin}m"
        )

    def _find_config(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            p = Path(get_package_share_directory("collision_manager")) / "config" / "collision_config.yaml"
            if p.exists():
                return str(p)
        except Exception:
            pass
        p = Path(__file__).parent.parent / "config" / "collision_config.yaml"
        return str(p) if p.exists() else ""

    def _load_config(self, path: str) -> dict:
        if not path:
            return {}
        with open(path) as f:
            return yaml.safe_load(f) or {}

    def _find_mesh_dir(self) -> Path:
        """Find FR3 collision mesh directory."""
        # Try package share
        try:
            from ament_index_python.packages import get_package_share_directory
            d = Path(get_package_share_directory("franka_description"))
            mesh_dir = d / "meshes" / "robots" / "fr3" / "collision"
            if mesh_dir.exists():
                return mesh_dir
        except Exception:
            pass
        # Try source
        src = Path(__file__).parent.parent.parent.parent / "franka_description" / "meshes" / "robots" / "fr3" / "collision"
        if src.exists():
            return src
        self.get_logger().error("FR3 collision mesh directory not found!")
        return Path(".")

    def _load_arm_meshes(self, side: str):
        """Load collision meshes for one FR3 arm."""
        loaded = 0
        for i in range(8):
            stl = self._mesh_dir / f"link{i}.stl"
            name = f"{side}_link{i}"
            if self.checker.add_mesh_body(name, str(stl)):
                loaded += 1
            else:
                self.get_logger().warn(f"Mesh not found: {stl}")
        self.get_logger().info(f"{side} arm: {loaded}/8 meshes loaded")

    def _load_environment(self):
        """Load environment obstacles from config."""
        env = self.config.get("environment", {})
        for name, obj in env.items():
            if obj.get("type") == "box":
                self.checker.add_box_obstacle(name, obj["size"], obj["position"])
                self.get_logger().info(f"Environment: {name}")

    def _build_exclude_pairs(self) -> set:
        """Build exclude pairs for same-arm adjacent/near links."""
        pairs = set()
        for side in ["left", "right"]:
            # Adjacent links (within 2 steps) on same arm
            for i in range(8):
                for j in range(i + 1, min(i + 3, 8)):
                    pairs.add((f"{side}_link{i}", f"{side}_link{j}"))

        # Cross-arm: exclude link0 with everything on the other arm (base is fixed)
        for i in range(8):
            pairs.add((f"left_link0", f"right_link{i}"))
            pairs.add((f"right_link0", f"left_link{i}"))
        # Also exclude link1 cross-arm with nearby links
        for i in range(3):
            pairs.add((f"left_link1", f"right_link{i}"))
            pairs.add((f"right_link1", f"left_link{i}"))

        # Base links vs table (mounted on table, always touching)
        for side in ["left", "right"]:
            pairs.add((f"{side}_link0", "table"))
            pairs.add((f"{side}_link1", "table"))

        return pairs

    def _get_tf(self, frame: str) -> Optional[np.ndarray]:
        """Get 4x4 transform from TF."""
        try:
            t = self._tf_buffer.lookup_transform(
                self._base_frame, frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.01))
            T = np.eye(4)
            T[0, 3] = t.transform.translation.x
            T[1, 3] = t.transform.translation.y
            T[2, 3] = t.transform.translation.z
            q = t.transform.rotation
            T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            return T
        except TransformException:
            return None

    def _update_arm_from_tf(self, side: str, tf_prefix: str = ""):
        """Update collision body transforms from TF tree."""
        for i in range(8):
            tf_frame = f"{tf_prefix}fr3_link{i}"
            body_name = f"{side}_link{i}"
            T = self._get_tf(tf_frame)
            if T is not None:
                self.checker.update_body_transform(body_name, T)

    def _enable_cb(self, request, response):
        self._enabled = request.data
        response.success = True
        response.message = f"Collision {'enabled' if self._enabled else 'disabled'}"
        return response

    def _check_loop(self):
        """Main loop: update transforms, check collision, publish."""
        if not self._enabled:
            return

        # Update from TF (dual-arm: left_fr3_linkX / right_fr3_linkX)
        self._update_arm_from_tf("left", tf_prefix="left_")
        self._update_arm_from_tf("right", tf_prefix="right_")

        # Check collision
        result = self.checker.check_collision()

        # Publish bool
        msg = Bool()
        msg.data = result.is_colliding
        self.collision_pub.publish(msg)

        # Publish markers
        self._publish_markers(result)

        # Log
        if result.is_colliding:
            self.get_logger().warn(
                f"COLLISION: {result.closest_pair[0]} <-> {result.closest_pair[1]} "
                f"dist={result.min_distance:.4f}m ({result.num_collisions} pairs)",
                throttle_duration_sec=1.0)

    def _publish_markers(self, result: CollisionResult):
        """Publish collision mesh bounding boxes as markers for RViz."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        colliding_names = set()
        for a, b, d in result.details:
            if d < 0:
                colliding_names.add(a)
                colliding_names.add(b)

        for idx, (name, body) in enumerate(self.checker.bodies.items()):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp
            m.ns = "collision_mesh"
            m.id = idx
            m.type = Marker.MESH_RESOURCE

            # Use the STL file as mesh resource
            # Find the link number from name (e.g., "left_link3" -> 3)
            link_num = name.split("_link")[-1]
            mesh_path = self._mesh_dir / f"link{link_num}.stl"
            if mesh_path.exists():
                m.mesh_resource = f"file://{mesh_path}"
            else:
                m.type = Marker.CUBE
                m.scale = Vector3(x=0.05, y=0.05, z=0.05)

            T = body.transform
            pos = T[:3, 3]
            m.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))

            quat = R.from_matrix(T[:3, :3]).as_quat()
            m.pose.orientation = Quaternion(
                x=float(quat[0]), y=float(quat[1]),
                z=float(quat[2]), w=float(quat[3]))

            m.scale = Vector3(x=1.0, y=1.0, z=1.0)

            if name in colliding_names:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            elif name.startswith("left"):
                m.color = ColorRGBA(r=0.0, g=0.8, b=0.3, a=0.3)
            else:
                m.color = ColorRGBA(r=0.3, g=0.3, b=0.8, a=0.3)

            m.lifetime.sec = 0
            m.lifetime.nanosec = 200000000
            ma.markers.append(m)

        self.marker_pub.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = CollisionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
