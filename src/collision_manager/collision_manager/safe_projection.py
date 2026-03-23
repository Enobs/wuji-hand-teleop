"""
Safe projection: project a target pose to the nearest collision-free pose.

When the virtual target is in collision, finds the closest safe pose
using binary search along the path from last safe pose to target.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R, Slerp

from .collision_checker import CollisionChecker


class SafeProjector:
    """Projects collision targets to nearest safe poses.

    Implements boundary sliding behavior: when target is in collision,
    the robot slides along the collision boundary instead of freezing.
    """

    def __init__(
        self,
        collision_checker: CollisionChecker,
        max_iterations: int = 10,
        step_back_ratio: float = 0.1,
    ):
        self.checker = collision_checker
        self.max_iterations = max_iterations
        self.step_back_ratio = step_back_ratio

        # Last known safe poses per arm
        self._last_safe: dict[str, np.ndarray] = {}

    def project_to_safe(
        self,
        arm_name: str,
        target_pose: np.ndarray,
        update_fn: callable,
    ) -> np.ndarray:
        """Project target pose to nearest collision-free pose.

        Args:
            arm_name: "left" or "right"
            target_pose: 4x4 target transform matrix
            update_fn: callable(pose) that updates collision body transforms
                       for the given arm to reflect the given EE pose

        Returns:
            4x4 safe transform matrix (may equal target if no collision)
        """
        # First check: is target already safe?
        update_fn(target_pose)
        result = self.checker.check_collision()

        if not result.is_colliding:
            self._last_safe[arm_name] = target_pose.copy()
            return target_pose

        # Target is in collision. Binary search between last safe and target.
        last_safe = self._last_safe.get(arm_name)
        if last_safe is None:
            # No previous safe pose, step back from target
            last_safe = target_pose.copy()
            last_safe[:3, 3] *= (1.0 - self.step_back_ratio)

        safe_pose = self._binary_search(
            last_safe, target_pose, update_fn
        )

        self._last_safe[arm_name] = safe_pose.copy()
        return safe_pose

    def _binary_search(
        self,
        safe_pose: np.ndarray,
        target_pose: np.ndarray,
        update_fn: callable,
    ) -> np.ndarray:
        """Binary search for the furthest safe pose along safe→target path."""
        lo = 0.0  # safe end
        hi = 1.0  # target end
        best_alpha = 0.0

        for _ in range(self.max_iterations):
            mid = (lo + hi) / 2.0
            interp = self._interpolate_pose(safe_pose, target_pose, mid)

            update_fn(interp)
            result = self.checker.check_collision()

            if result.is_colliding:
                hi = mid  # too far, step back
            else:
                lo = mid  # safe, try further
                best_alpha = mid

        return self._interpolate_pose(safe_pose, target_pose, best_alpha)

    @staticmethod
    def _interpolate_pose(
        pose_a: np.ndarray, pose_b: np.ndarray, alpha: float
    ) -> np.ndarray:
        """Interpolate between two 4x4 poses (linear pos + slerp rot)."""
        result = np.eye(4)

        # Linear interpolation for position
        result[:3, 3] = (1.0 - alpha) * pose_a[:3, 3] + alpha * pose_b[:3, 3]

        # SLERP for rotation
        r_a = R.from_matrix(pose_a[:3, :3])
        r_b = R.from_matrix(pose_b[:3, :3])

        # Use scipy Slerp
        key_rots = R.concatenate([r_a, r_b])
        slerp = Slerp([0.0, 1.0], key_rots)
        result[:3, :3] = slerp(alpha).as_matrix()

        return result

    def reset(self, arm_name: Optional[str] = None):
        """Reset cached safe poses."""
        if arm_name:
            self._last_safe.pop(arm_name, None)
        else:
            self._last_safe.clear()
