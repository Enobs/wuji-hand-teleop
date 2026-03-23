"""
Core collision checking using FCL with capsule geometry.

Provides real-time collision detection and distance queries
for dual-arm + hand configurations.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

try:
    import fcl
    FCL_AVAILABLE = True
except ImportError:
    FCL_AVAILABLE = False


@dataclass
class CapsuleDef:
    """Capsule geometry definition."""
    radius: float
    length: float


@dataclass
class CollisionBody:
    """A named collision body with FCL object."""
    name: str
    capsule: CapsuleDef
    fcl_geom: object = None
    fcl_obj: object = None
    transform: np.ndarray = field(default_factory=lambda: np.eye(4))

    def __post_init__(self):
        if FCL_AVAILABLE:
            self.fcl_geom = fcl.Cylinder(self.capsule.radius, self.capsule.length)
            self.fcl_obj = fcl.CollisionObject(self.fcl_geom, fcl.Transform())
            self.update_transform(self.transform)

    def update_transform(self, T: np.ndarray):
        """Update FCL object pose from 4x4 transform matrix."""
        self.transform = T
        if self.fcl_obj is not None:
            R = T[:3, :3]
            t = T[:3, 3]
            self.fcl_obj.setTransform(fcl.Transform(R, t))


@dataclass
class CollisionResult:
    """Result of a collision check."""
    is_colliding: bool
    min_distance: float
    closest_pair: Tuple[str, str] = ("", "")
    nearest_safe_point: Optional[np.ndarray] = None
    num_collisions: int = 0
    details: List[Tuple[str, str, float]] = field(default_factory=list)


class CollisionChecker:
    """FCL-based collision checker for dual-arm teleoperation.

    Uses simplified capsule (cylinder) geometry for real-time performance.
    Checks collision between:
    - Left arm links vs Right arm links
    - Arm links vs hand
    - All links vs environment (table, etc.)
    """

    def __init__(
        self,
        safety_margin: float = 0.02,
        exclude_pairs: Optional[List[Tuple[str, str]]] = None,
    ):
        if not FCL_AVAILABLE:
            raise ImportError(
                "python-fcl not installed. Run: pip install python-fcl"
            )

        self.safety_margin = safety_margin
        self.exclude_pairs = set()
        if exclude_pairs:
            for a, b in exclude_pairs:
                self.exclude_pairs.add((a, b))
                self.exclude_pairs.add((b, a))

        # Collision bodies indexed by name
        self.bodies: Dict[str, CollisionBody] = {}

        # Environment objects (static)
        self.env_objects: List[fcl.CollisionObject] = []
        self.env_names: List[str] = []

    def add_body(self, name: str, capsule: CapsuleDef) -> None:
        """Register a collision body (arm link or hand link)."""
        self.bodies[name] = CollisionBody(name=name, capsule=capsule)

    def add_box_obstacle(
        self, name: str, size: List[float], position: List[float]
    ) -> None:
        """Add a static box obstacle (e.g., table)."""
        box = fcl.Box(*size)
        T = fcl.Transform(np.eye(3), np.array(position))
        obj = fcl.CollisionObject(box, T)
        self.env_objects.append(obj)
        self.env_names.append(name)

    def update_body_transform(self, name: str, T: np.ndarray) -> None:
        """Update the world-frame transform of a collision body."""
        if name in self.bodies:
            self.bodies[name].update_transform(T)

    def update_arm_transforms(
        self, prefix: str, transforms: Dict[str, np.ndarray]
    ) -> None:
        """Update transforms for all links of an arm.

        Args:
            prefix: "left" or "right"
            transforms: dict mapping link name to 4x4 transform
        """
        for link_name, T in transforms.items():
            full_name = f"{prefix}_{link_name}"
            if full_name in self.bodies:
                self.bodies[full_name].update_transform(T)

    def _should_check_pair(self, name_a: str, name_b: str) -> bool:
        """Check if this pair should be tested for collision."""
        if name_a == name_b:
            return False
        if (name_a, name_b) in self.exclude_pairs:
            return False

        # Don't check adjacent links on the same arm
        # (exclude_pairs should handle this, but double check)
        a_parts = name_a.split("_", 1)
        b_parts = name_b.split("_", 1)
        if len(a_parts) == 2 and len(b_parts) == 2:
            if a_parts[0] == b_parts[0]:  # same arm prefix
                # Same arm adjacent links already in exclude_pairs
                pass

        return True

    def check_collision(self) -> CollisionResult:
        """Check all collision pairs. Returns CollisionResult."""
        body_list = list(self.bodies.values())
        n = len(body_list)

        min_dist = float('inf')
        closest_pair = ("", "")
        is_colliding = False
        num_collisions = 0
        details = []

        # Check body-body pairs
        for i in range(n):
            for j in range(i + 1, n):
                a = body_list[i]
                b = body_list[j]

                if not self._should_check_pair(a.name, b.name):
                    continue

                # Distance query
                request = fcl.DistanceRequest(enable_nearest_points=True)
                result = fcl.DistanceResult()
                dist = fcl.distance(a.fcl_obj, b.fcl_obj, request, result)

                # Apply safety margin
                effective_dist = dist - self.safety_margin

                if effective_dist < 0:
                    is_colliding = True
                    num_collisions += 1

                if effective_dist < min_dist:
                    min_dist = effective_dist
                    closest_pair = (a.name, b.name)

                if effective_dist < self.safety_margin * 2:
                    details.append((a.name, b.name, effective_dist))

        # Check body-environment pairs
        for body in body_list:
            for env_idx, env_obj in enumerate(self.env_objects):
                request = fcl.DistanceRequest(enable_nearest_points=True)
                result = fcl.DistanceResult()
                dist = fcl.distance(body.fcl_obj, env_obj, request, result)

                effective_dist = dist - self.safety_margin

                if effective_dist < 0:
                    is_colliding = True
                    num_collisions += 1

                if effective_dist < min_dist:
                    min_dist = effective_dist
                    closest_pair = (body.name, self.env_names[env_idx])

                if effective_dist < self.safety_margin * 2:
                    details.append(
                        (body.name, self.env_names[env_idx], effective_dist)
                    )

        return CollisionResult(
            is_colliding=is_colliding,
            min_distance=min_dist,
            closest_pair=closest_pair,
            num_collisions=num_collisions,
            details=details,
        )

    def get_distance(self, name_a: str, name_b: str) -> float:
        """Get distance between two specific bodies."""
        if name_a not in self.bodies or name_b not in self.bodies:
            return float('inf')

        a = self.bodies[name_a]
        b = self.bodies[name_b]

        request = fcl.DistanceRequest()
        result = fcl.DistanceResult()
        return fcl.distance(a.fcl_obj, b.fcl_obj, request, result)
