"""
Collision Manager ROS2 Node

Subscribes to joint states, checks collision at 100Hz,
publishes collision state and safe projected poses.

Can run standalone with simulated joints for testing
(use joint_state_publisher_gui).
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
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion
from visualization_msgs.msg import Marker, MarkerArray

from .collision_checker import CollisionChecker, CapsuleDef, CollisionResult
from .fr3_kinematics import fr3_forward_kinematics, FR3_HOME_JOINTS, check_joint_limits

import tf2_ros
from tf2_ros import TransformException


class CollisionManagerNode(Node):
    """Real-time collision detection node for dual-arm teleoperation."""

    def __init__(self):
        super().__init__("collision_manager")

        # Load config
        config_path = self._find_config()
        self.config = self._load_config(config_path)

        self.get_logger().info(f"Loaded config from: {config_path}")

        # Initialize collision checker
        self.checker = CollisionChecker(
            safety_margin=self.config.get("safety_margin", 0.02),
            exclude_pairs=self._build_exclude_pairs(),
        )

        # Register collision bodies
        self._register_franka_bodies("left")
        self._register_franka_bodies("right")
        self._register_hand_bodies()
        self._register_environment()

        # QoS
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Publishers
        self.collision_pub = self.create_publisher(Bool, "/collision_state", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/collision_markers", 10)

        # Subscribers - joint states from both arms
        self.create_subscription(
            JointState, "/franka_left/joint_states",
            self._left_joint_cb, qos)
        self.create_subscription(
            JointState, "/franka_right/joint_states",
            self._right_joint_cb, qos)
        # Also accept simulated joint states (for testing without hardware)
        self.create_subscription(
            JointState, "/joint_states",
            self._sim_joint_cb, qos)

        # Service to enable/disable
        self._enabled = True
        self.create_service(
            SetBool, "/collision/enable", self._enable_cb)

        # State (default to FR3 home position)
        self._left_joints: Optional[np.ndarray] = FR3_HOME_JOINTS.copy()
        self._right_joints: Optional[np.ndarray] = FR3_HOME_JOINTS.copy()

        # TF listener - use TF from robot_state_publisher when available
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._use_tf = True  # prefer TF over FK
        self._tf_base_frame = "world"
        # FR3 link names in URDF
        self._fr3_link_names = [
            "fr3_link0", "fr3_link1", "fr3_link2", "fr3_link3",
            "fr3_link4", "fr3_link5", "fr3_link6", "fr3_link7",
            "fr3_link8",
        ]
        self._last_result: Optional[CollisionResult] = None

        # Timer
        rate = self.config.get("check_rate_hz", 100.0)
        self.create_timer(1.0 / rate, self._check_loop)

        body_count = len(self.checker.bodies)
        env_count = len(self.checker.env_objects)
        self.get_logger().info(
            f"Collision manager ready: {body_count} bodies, "
            f"{env_count} env objects, {rate}Hz"
        )

    def _find_config(self) -> str:
        """Find collision config file."""
        # Try package share directory first
        try:
            from ament_index_python.packages import get_package_share_directory
            share = Path(get_package_share_directory("collision_manager"))
            path = share / "config" / "collision_config.yaml"
            if path.exists():
                return str(path)
        except Exception:
            pass

        # Fallback: relative to source
        src_path = Path(__file__).parent.parent / "config" / "collision_config.yaml"
        if src_path.exists():
            return str(src_path)

        self.get_logger().warn("Config not found, using defaults")
        return ""

    def _load_config(self, path: str) -> dict:
        if not path:
            return {}
        with open(path, 'r') as f:
            return yaml.safe_load(f) or {}

    def _build_exclude_pairs(self) -> List:
        """Build exclude pairs with arm prefixes.

        Also auto-excludes all same-arm pairs that are within 2 links
        of each other (can't physically collide).
        """
        raw = self.config.get("exclude_pairs", [])
        pairs = []
        for a, b in raw:
            pairs.append((f"left_{a}", f"left_{b}"))
            pairs.append((f"right_{a}", f"right_{b}"))

        # Auto-exclude same-arm links within 2 steps (physically impossible)
        link_names = ["link0", "link1", "link2", "link3",
                      "link4", "link5", "link6", "link7", "hand"]
        for side in ["left", "right"]:
            for i in range(len(link_names)):
                for j in range(i + 1, min(i + 3, len(link_names))):
                    a = f"{side}_{link_names[i]}"
                    b = f"{side}_{link_names[j]}"
                    pairs.append((a, b))
                    for suffix_a in ["", "_0", "_1"]:
                        for suffix_b in ["", "_0", "_1"]:
                            pairs.append((a + suffix_a, b + suffix_b))

        # Exclude cross-arm base links (fixed, always overlapping due to spacing)
        pairs.append(("left_link0", "right_link0"))
        pairs.append(("left_link0", "right_link1"))
        pairs.append(("right_link0", "left_link1"))

        return pairs

    def _register_franka_bodies(self, side: str):
        """Register capsule bodies for one Franka arm."""
        capsules = self.config.get("franka_capsules", {})
        self._capsule_offsets = getattr(self, '_capsule_offsets', {})

        for link_name, params in capsules.items():
            if isinstance(params, list):
                for i, p in enumerate(params):
                    name = f"{side}_{link_name}_{i}" if len(params) > 1 else f"{side}_{link_name}"
                    cap = CapsuleDef(radius=p["radius"], length=p["length"])
                    self.checker.add_body(name, cap)
                    # Store local offset for transform computation
                    self._capsule_offsets[name] = {
                        "link": link_name,
                        "xyz": p.get("xyz", [0, 0, 0]),
                    }
            else:
                name = f"{side}_{link_name}"
                cap = CapsuleDef(radius=params["radius"], length=params["length"])
                self.checker.add_body(name, cap)
                self._capsule_offsets[name] = {
                    "link": link_name,
                    "xyz": params.get("xyz", [0, 0, 0]),
                }

    def _register_hand_bodies(self):
        """Register Wuji Hand collision bodies (right hand only for now)."""
        capsules = self.config.get("wuji_hand_capsules", {})
        for link_name, params in capsules.items():
            name = f"right_hand_{link_name}"
            cap = CapsuleDef(
                radius=params["radius"],
                length=params["length"],
            )
            self.checker.add_body(name, cap)

    def _register_environment(self):
        """Register static environment obstacles."""
        env = self.config.get("environment", {})
        for name, obj in env.items():
            if obj.get("type") == "box":
                self.checker.add_box_obstacle(
                    name,
                    obj["size"],
                    obj["position"],
                )
                self.get_logger().info(f"Added environment: {name} (box)")

    def _left_joint_cb(self, msg: JointState):
        if msg.position:
            self._left_joints = np.array(msg.position)

    def _right_joint_cb(self, msg: JointState):
        if msg.position:
            self._right_joints = np.array(msg.position)

    def _sim_joint_cb(self, msg: JointState):
        """Accept /joint_states for simulation testing."""
        if msg.position:
            joints = np.array(msg.position)
            n = len(joints)
            if n >= 14:
                self._left_joints = joints[:7]
                self._right_joints = joints[7:14]
            elif n >= 7:
                # Single arm: use same joints for both arms (testing)
                self._left_joints = joints[:7]
                self._right_joints = joints[:7]

    def _enable_cb(self, request, response):
        self._enabled = request.data
        response.success = True
        response.message = f"Collision checking {'enabled' if self._enabled else 'disabled'}"
        self.get_logger().info(response.message)
        return response

    def _joints_to_transforms(
        self, side: str, joints: np.ndarray
    ) -> Dict[str, np.ndarray]:
        """Convert joint angles to link transforms using FR3 FK.

        Uses official FR3 kinematics parameters from fr3/kinematics.yaml.
        """
        base_offset = self.config.get("dual_arm", {})
        offset = np.array(
            base_offset.get(f"{side}_base_offset", [0, 0, 0])
        )

        base_T = np.eye(4)
        base_T[:3, 3] = offset

        # Ensure we have exactly 7 joints
        q = np.zeros(7)
        q[:min(len(joints), 7)] = joints[:7]

        return fr3_forward_kinematics(q, base_transform=base_T)

    def _get_transforms_from_tf(self, prefix: str = "") -> Optional[Dict[str, np.ndarray]]:
        """Get link transforms from TF tree (published by robot_state_publisher)."""
        transforms = {}
        # Map our internal names to URDF link names
        link_map = {
            "link0": f"{prefix}fr3_link0",
            "link1": f"{prefix}fr3_link1",
            "link2": f"{prefix}fr3_link2",
            "link3": f"{prefix}fr3_link3",
            "link4": f"{prefix}fr3_link4",
            "link5": f"{prefix}fr3_link5",
            "link6": f"{prefix}fr3_link6",
            "link7": f"{prefix}fr3_link7",
            "hand":  f"{prefix}fr3_link8",
        }

        for our_name, tf_name in link_map.items():
            try:
                t = self._tf_buffer.lookup_transform(
                    self._tf_base_frame, tf_name,
                    rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=0.01))

                T = np.eye(4)
                T[0, 3] = t.transform.translation.x
                T[1, 3] = t.transform.translation.y
                T[2, 3] = t.transform.translation.z

                q = t.transform.rotation
                from scipy.spatial.transform import Rotation as R
                T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

                transforms[our_name] = T
            except TransformException:
                return None  # TF not ready

        return transforms

    def _update_capsule_transforms(self, side: str, link_transforms: Dict):
        """Update all capsule body transforms, applying local offsets."""
        for body_name, offset_info in self._capsule_offsets.items():
            if not body_name.startswith(side):
                continue
            link_name = offset_info["link"]
            if link_name not in link_transforms:
                continue

            T_link = link_transforms[link_name]
            xyz = offset_info["xyz"]

            # Apply local offset in link frame
            T_capsule = T_link.copy()
            T_capsule[:3, 3] += T_link[:3, :3] @ np.array(xyz)

            self.checker.update_body_transform(body_name, T_capsule)

    def _check_loop(self):
        """Main collision check loop at configured rate."""
        if not self._enabled:
            return

        # Try TF first (from robot_state_publisher), fallback to FK
        tf_transforms = self._get_transforms_from_tf() if self._use_tf else None

        if tf_transforms is not None:
            # Use TF for left arm (single arm mode: same for both)
            self._update_capsule_transforms("left", tf_transforms)
            self._update_capsule_transforms("right", tf_transforms)
        else:
            # Fallback to FK from joint states
            if self._left_joints is not None:
                transforms = self._joints_to_transforms("left", self._left_joints)
                self._update_capsule_transforms("left", transforms)

            if self._right_joints is not None:
                transforms = self._joints_to_transforms("right", self._right_joints)
                self._update_capsule_transforms("right", transforms)

        # Run collision check
        result = self.checker.check_collision()
        self._last_result = result

        # Publish collision state
        msg = Bool()
        msg.data = result.is_colliding
        self.collision_pub.publish(msg)

        # Publish visualization markers
        self._publish_markers(result)

        # Log warnings
        if result.is_colliding:
            self.get_logger().warn(
                f"COLLISION: {result.closest_pair[0]} <-> {result.closest_pair[1]} "
                f"dist={result.min_distance:.4f}m ({result.num_collisions} pairs)",
                throttle_duration_sec=1.0,
            )
        else:
            self.get_logger().debug(
                f"Safe: min_dist={result.min_distance:.4f}m "
                f"closest={result.closest_pair}",
                throttle_duration_sec=5.0,
            )


    def _publish_markers(self, result: CollisionResult):
        """Publish collision capsules as RViz markers."""
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        # Colliding body names for coloring
        colliding_names = set()
        for a, b, d in result.details:
            if d < 0:
                colliding_names.add(a)
                colliding_names.add(b)

        for idx, (name, body) in enumerate(self.checker.bodies.items()):
            m = Marker()
            m.header.frame_id = "world"
            m.header.stamp = stamp
            m.ns = "collision_capsules"
            m.id = idx
            m.type = Marker.CYLINDER
            m.action = Marker.ADD

            T = body.transform
            pos = T[:3, 3]
            m.pose.position = Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2]))

            # Extract quaternion from rotation matrix
            from scipy.spatial.transform import Rotation as R
            quat = R.from_matrix(T[:3, :3]).as_quat()  # [x,y,z,w]
            m.pose.orientation = Quaternion(
                x=float(quat[0]), y=float(quat[1]),
                z=float(quat[2]), w=float(quat[3]))

            r = body.capsule.radius
            l = body.capsule.length
            m.scale = Vector3(x=r * 2, y=r * 2, z=l)

            # Green = safe, Red = colliding
            if name in colliding_names:
                m.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.5)
            elif name.startswith("left"):
                m.color = ColorRGBA(r=0.0, g=0.7, b=0.3, a=0.3)
            else:
                m.color = ColorRGBA(r=0.3, g=0.3, b=0.7, a=0.3)

            m.lifetime.sec = 0
            m.lifetime.nanosec = 200000000  # 200ms
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
