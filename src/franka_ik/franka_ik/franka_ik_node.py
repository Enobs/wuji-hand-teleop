"""
Franka FR3 Dual-Arm IK Node with Integrated Safe Projection

Pipeline per control cycle:
  1. pink IK → candidate joint config q_next
  2. Pinocchio FK(q_next) → all link poses
  3. FCL collision check
  4. Safe: accept q_next
     Collision: binary search [last_safe_q, q_next] → nearest safe q
  5. Publish /joint_states

Subscribes:
  /left_arm_target_pose  (geometry_msgs/PoseStamped)
  /right_arm_target_pose (geometry_msgs/PoseStamped)

Publishes:
  /joint_states          (sensor_msgs/JointState)
  /collision_state       (std_msgs/Bool)
"""
from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import pinocchio as pin
import pink
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from scipy.spatial.transform import Rotation as R

from collision_manager.mesh_collision_checker import MeshCollisionChecker


LEFT_EE_FRAME  = "left_fr3_link8"
RIGHT_EE_FRAME = "right_fr3_link8"

LEFT_JOINTS  = [f"left_fr3_joint{i}"  for i in range(1, 8)]
RIGHT_JOINTS = [f"right_fr3_joint{i}" for i in range(1, 8)]

# FR3 home pose (elbow up)
FR3_HOME = np.array([0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785])

# Link names in collision checker  (left_link0..7, right_link0..7)
_SIDES = ["left", "right"]
_LINK_INDICES = range(8)  # link0 ~ link7 (link8 is EE, no collision mesh)


def _find_urdf() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        p = Path(get_package_share_directory("franka_description")) / "robots" / "fr3" / "fr3_dual.urdf"
        if p.exists():
            return str(p)
    except Exception:
        pass
    p = Path(__file__).parents[3] / "franka_description" / "robots" / "fr3" / "fr3_dual.urdf"
    return str(p)


def _find_mesh_dir() -> Path:
    try:
        from ament_index_python.packages import get_package_share_directory
        d = Path(get_package_share_directory("franka_description")) / "meshes" / "robots" / "fr3" / "collision"
        if d.exists():
            return d
    except Exception:
        pass
    d = Path(__file__).parents[3] / "franka_description" / "meshes" / "robots" / "fr3" / "collision"
    return d


def _pose_to_se3(msg: PoseStamped) -> pin.SE3:
    p = msg.pose.position
    o = msg.pose.orientation
    rot = R.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    return pin.SE3(rot, np.array([p.x, p.y, p.z]))


def _build_exclude_pairs() -> set:
    """Same-arm adjacent links + base links vs table."""
    pairs = set()
    for side in _SIDES:
        for i in _LINK_INDICES:
            for j in range(i + 1, min(i + 4, 8)):
                pairs.add((f"{side}_link{i}", f"{side}_link{j}"))
    # base links are fixed — skip cross-arm checks for link0/1
    for i in _LINK_INDICES:
        pairs.add(("left_link0", f"right_link{i}"))
        pairs.add((f"right_link0", f"left_link{i}"))
    for i in range(3):
        pairs.add(("left_link1", f"right_link{i}"))
        pairs.add(("right_link1", f"left_link{i}"))
    return pairs


class FrankaIKNode(Node):

    def __init__(self):
        super().__init__("franka_ik_node")

        self.declare_parameter("dt",            0.01)
        self.declare_parameter("rate_hz",       100.0)
        self.declare_parameter("pos_cost",      1.0)
        self.declare_parameter("ori_cost",      0.5)
        self.declare_parameter("solver",        "quadprog")
        self.declare_parameter("safety_margin", 0.03)
        self.declare_parameter("bisect_iters",  8)
        self.declare_parameter("enable_collision", True)

        self._dt             = self.get_parameter("dt").value
        rate_hz              = self.get_parameter("rate_hz").value
        pos_cost             = self.get_parameter("pos_cost").value
        ori_cost             = self.get_parameter("ori_cost").value
        self._solver         = self.get_parameter("solver").value
        safety_margin        = self.get_parameter("safety_margin").value
        self._bisect_iters   = self.get_parameter("bisect_iters").value
        self._collision_on   = self.get_parameter("enable_collision").value

        # ── Pinocchio model ──────────────────────────────────────────────
        urdf = _find_urdf()
        self.get_logger().info(f"URDF: {urdf}")
        self._model = pin.buildModelFromUrdf(urdf)
        self._data  = self._model.createData()

        q0 = pin.neutral(self._model)
        for jname, val in zip(LEFT_JOINTS + RIGHT_JOINTS, list(FR3_HOME) * 2):
            jid = self._model.getJointId(jname)
            if jid < len(self._model.joints):
                q0[self._model.joints[jid].idx_q] = val

        self._configuration = pink.Configuration(self._model, self._data, q0)
        self._last_safe_q   = q0.copy()

        # ── pink tasks ───────────────────────────────────────────────────
        self._left_task  = pink.tasks.FrameTask(
            LEFT_EE_FRAME,  position_cost=pos_cost, orientation_cost=ori_cost)
        self._right_task = pink.tasks.FrameTask(
            RIGHT_EE_FRAME, position_cost=pos_cost, orientation_cost=ori_cost)
        self._posture_task = pink.tasks.PostureTask(cost=1e-3)
        self._posture_task.set_target(q0)

        self._configuration.update()
        self._left_task.set_target(
            self._configuration.get_transform_frame_to_world(LEFT_EE_FRAME))
        self._right_task.set_target(
            self._configuration.get_transform_frame_to_world(RIGHT_EE_FRAME))

        self._limits = [pink.limits.ConfigurationLimit(self._model)]

        # ── FCL collision checker ────────────────────────────────────────
        self._checker: Optional[MeshCollisionChecker] = None
        if self._collision_on:
            self._checker = MeshCollisionChecker(
                safety_margin=safety_margin,
                exclude_pairs=_build_exclude_pairs(),
            )
            mesh_dir = _find_mesh_dir()
            loaded = 0
            for side in _SIDES:
                for i in _LINK_INDICES:
                    stl = mesh_dir / f"link{i}.stl"
                    if self._checker.add_mesh_body(f"{side}_link{i}", str(stl)):
                        loaded += 1
            self.get_logger().info(
                f"Collision: {loaded}/{len(_SIDES)*len(_LINK_INDICES)} meshes loaded, "
                f"margin={safety_margin}m")

        # Precompute frame IDs for FK→collision update
        self._frame_ids: dict[str, int] = {}
        for side in _SIDES:
            for i in _LINK_INDICES:
                fname = f"{side}_fr3_link{i}"
                fid = self._model.getFrameId(fname)
                if fid < len(self._model.frames):
                    self._frame_ids[f"{side}_link{i}"] = fid

        # ── ROS2 I/O ────────────────────────────────────────────────────
        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(PoseStamped, "/left_arm_target_pose",
                                 lambda m: setattr(self, "_left_target",  _pose_to_se3(m)), qos)
        self.create_subscription(PoseStamped, "/right_arm_target_pose",
                                 lambda m: setattr(self, "_right_target", _pose_to_se3(m)), qos)

        self._left_target:  Optional[pin.SE3] = None
        self._right_target: Optional[pin.SE3] = None

        self._js_pub        = self.create_publisher(JointState, "/joint_states", 10)
        self._collision_pub = self.create_publisher(Bool, "/collision_state", 10)

        self.create_timer(1.0 / rate_hz, self._loop)
        self.get_logger().info(
            f"FrankaIKNode ready — {rate_hz}Hz, solver={self._solver}, "
            f"collision={'ON' if self._collision_on else 'OFF'}")

    # ── main loop ────────────────────────────────────────────────────────

    def _loop(self):
        if self._left_target is not None:
            self._left_task.set_target(self._left_target)
        if self._right_target is not None:
            self._right_task.set_target(self._right_target)

        # 1. Solve IK → candidate velocity
        try:
            velocity = pink.solve_ik(
                self._configuration,
                [self._left_task, self._right_task, self._posture_task],
                self._dt,
                solver=self._solver,
                limits=self._limits,
            )
        except Exception as e:
            self.get_logger().warn(f"IK failed: {e}", throttle_duration_sec=2.0)
            return

        # 2. Candidate q
        q_next = pin.integrate(self._model, self._configuration.q, velocity * self._dt)

        # 3. Collision check + safe projection
        is_colliding = False
        if self._checker is not None:
            self._update_collision_from_q(q_next)
            result = self._checker.check_collision()
            is_colliding = result.is_colliding

            if is_colliding:
                self.get_logger().warn(
                    f"Collision: {result.closest_pair[0]} ↔ {result.closest_pair[1]} "
                    f"dist={result.min_distance:.3f}m",
                    throttle_duration_sec=0.5)
                q_next = self._bisect_safe(self._last_safe_q, q_next)
            else:
                self._last_safe_q = q_next.copy()
        else:
            self._last_safe_q = q_next.copy()

        # 4. Accept q_next
        self._configuration.q = q_next
        pin.forwardKinematics(self._model, self._data, q_next)

        # 5. Publish
        self._publish_js(q_next)
        msg = Bool()
        msg.data = is_colliding
        self._collision_pub.publish(msg)

    # ── collision helpers ────────────────────────────────────────────────

    def _update_collision_from_q(self, q: np.ndarray):
        """Run FK for q and push all link transforms into FCL."""
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        for body_name, fid in self._frame_ids.items():
            oMf = self._data.oMf[fid]
            T = np.eye(4)
            T[:3, :3] = oMf.rotation
            T[:3, 3]  = oMf.translation
            self._checker.update_body_transform(body_name, T)

    def _bisect_safe(self, q_safe: np.ndarray, q_target: np.ndarray) -> np.ndarray:
        """Binary search in joint space for the furthest collision-free config."""
        lo, hi   = 0.0, 1.0
        best_alpha = 0.0

        for _ in range(self._bisect_iters):
            mid = (lo + hi) / 2.0
            q_mid = pin.interpolate(self._model, q_safe, q_target, mid)
            self._update_collision_from_q(q_mid)
            if self._checker.check_collision().is_colliding:
                hi = mid
            else:
                lo = mid
                best_alpha = mid

        return pin.interpolate(self._model, q_safe, q_target, best_alpha)

    # ── publish ──────────────────────────────────────────────────────────

    def _publish_js(self, q: np.ndarray):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = LEFT_JOINTS + RIGHT_JOINTS
        positions = []
        for jname in LEFT_JOINTS + RIGHT_JOINTS:
            jid = self._model.getJointId(jname)
            positions.append(float(q[self._model.joints[jid].idx_q]))
        msg.position = positions
        self._js_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FrankaIKNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
