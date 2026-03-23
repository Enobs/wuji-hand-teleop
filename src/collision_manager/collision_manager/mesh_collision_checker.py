"""
Mesh-based collision checking using FCL.

Loads collision meshes from URDF STL files for accurate
real-time collision detection.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np
import trimesh
import fcl


@dataclass
class CollisionResult:
    """Result of a collision check."""
    is_colliding: bool
    min_distance: float
    closest_pair: Tuple[str, str] = ("", "")
    num_collisions: int = 0
    details: List[Tuple[str, str, float]] = field(default_factory=list)


class MeshCollisionBody:
    """A collision body loaded from STL mesh."""

    def __init__(self, name: str, stl_path: str, scale: float = 1.0):
        self.name = name
        mesh = trimesh.load(stl_path)
        verts = np.array(mesh.vertices * scale, dtype=np.float64)
        tris = np.array(mesh.faces, dtype=np.int32)

        self.bvh = fcl.BVHModel()
        self.bvh.beginModel(len(tris), len(verts))
        self.bvh.addSubModel(verts, tris)
        self.bvh.endModel()

        self.fcl_obj = fcl.CollisionObject(self.bvh, fcl.Transform())
        self.transform = np.eye(4)
        self.face_count = len(tris)

    def update_transform(self, T: np.ndarray):
        """Update pose from 4x4 transform matrix."""
        self.transform = T
        R = T[:3, :3]
        t = T[:3, 3]
        self.fcl_obj.setTransform(fcl.Transform(R, t))


class BoxCollisionBody:
    """A static box obstacle (e.g., table)."""

    def __init__(self, name: str, size: List[float], position: List[float]):
        self.name = name
        box = fcl.Box(*size)
        T = fcl.Transform(np.eye(3), np.array(position))
        self.fcl_obj = fcl.CollisionObject(box, T)


class MeshCollisionChecker:
    """FCL collision checker using URDF collision meshes.

    Loads STL files for each link, uses TF transforms for positioning.
    """

    def __init__(
        self,
        safety_margin: float = 0.03,
        exclude_pairs: Optional[set] = None,
    ):
        self.safety_margin = safety_margin
        self.exclude_pairs = exclude_pairs or set()

        self.bodies: Dict[str, MeshCollisionBody] = {}
        self.env_bodies: Dict[str, BoxCollisionBody] = {}

    def add_mesh_body(self, name: str, stl_path: str, scale: float = 1.0) -> bool:
        """Load and register a collision mesh."""
        path = Path(stl_path)
        if not path.exists():
            return False
        self.bodies[name] = MeshCollisionBody(name, str(path), scale)
        return True

    def add_box_obstacle(self, name: str, size: List[float], position: List[float]):
        """Add a static box obstacle."""
        self.env_bodies[name] = BoxCollisionBody(name, size, position)

    def update_body_transform(self, name: str, T: np.ndarray):
        """Update world-frame transform for a body."""
        if name in self.bodies:
            self.bodies[name].update_transform(T)

    def _should_check(self, a: str, b: str) -> bool:
        if a == b:
            return False
        if (a, b) in self.exclude_pairs or (b, a) in self.exclude_pairs:
            return False
        # Also check with simplified names (for env objects)
        return True

    def add_exclude_pair(self, a: str, b: str):
        """Add a pair to exclude from checking."""
        self.exclude_pairs.add((a, b))
        self.exclude_pairs.add((b, a))

    def check_collision(self) -> CollisionResult:
        """Check all collision pairs."""
        body_list = list(self.bodies.values())
        n = len(body_list)

        min_dist = float('inf')
        closest_pair = ("", "")
        is_colliding = False
        num_collisions = 0
        details = []

        req = fcl.DistanceRequest(enable_nearest_points=True)

        # Body-body pairs
        for i in range(n):
            for j in range(i + 1, n):
                a = body_list[i]
                b = body_list[j]

                if not self._should_check(a.name, b.name):
                    continue

                res = fcl.DistanceResult()
                dist = fcl.distance(a.fcl_obj, b.fcl_obj, req, res)

                effective = dist - self.safety_margin
                if effective < 0:
                    is_colliding = True
                    num_collisions += 1

                if effective < min_dist:
                    min_dist = effective
                    closest_pair = (a.name, b.name)

                if effective < self.safety_margin * 2:
                    details.append((a.name, b.name, effective))

        # Body-environment pairs
        for body in body_list:
            for env_name, env_body in self.env_bodies.items():
                if not self._should_check(body.name, env_name):
                    continue
                res = fcl.DistanceResult()
                dist = fcl.distance(body.fcl_obj, env_body.fcl_obj, req, res)

                effective = dist - self.safety_margin
                if effective < 0:
                    is_colliding = True
                    num_collisions += 1

                if effective < min_dist:
                    min_dist = effective
                    closest_pair = (body.name, env_name)

                if effective < self.safety_margin * 2:
                    details.append((body.name, env_name, effective))

        return CollisionResult(
            is_colliding=is_colliding,
            min_distance=min_dist,
            closest_pair=closest_pair,
            num_collisions=num_collisions,
            details=details,
        )
