"""
Franka FR3 Forward Kinematics

Computes link transforms from joint angles using the official
FR3 kinematics parameters (from fr3/kinematics.yaml).

Modified DH convention matching Franka's URDF.
"""
from __future__ import annotations

from typing import Dict

import numpy as np


# FR3 DH-like parameters from kinematics.yaml
# Each joint: (x, y, z, roll, pitch, yaw) relative to parent
_FR3_JOINT_PARAMS = [
    # joint1: base to link1
    {"xyz": [0, 0, 0.333], "rpy": [0, 0, 0]},
    # joint2: link1 to link2
    {"xyz": [0, 0, 0], "rpy": [-np.pi / 2, 0, 0]},
    # joint3: link2 to link3
    {"xyz": [0, -0.316, 0], "rpy": [np.pi / 2, 0, 0]},
    # joint4: link3 to link4
    {"xyz": [0.0825, 0, 0], "rpy": [np.pi / 2, 0, 0]},
    # joint5: link4 to link5
    {"xyz": [-0.0825, 0.384, 0], "rpy": [-np.pi / 2, 0, 0]},
    # joint6: link5 to link6
    {"xyz": [0, 0, 0], "rpy": [np.pi / 2, 0, 0]},
    # joint7: link6 to link7
    {"xyz": [0.088, 0, 0], "rpy": [np.pi / 2, 0, 0]},
    # flange: link7 to EE
    {"xyz": [0, 0, 0.107], "rpy": [0, 0, 0]},
]

# Joint limits from fr3/joint_limits.yaml
FR3_JOINT_LIMITS = np.array([
    [-2.9007, 2.9007],   # joint1
    [-1.8361, 1.8361],   # joint2
    [-2.9007, 2.9007],   # joint3
    [-3.0770, -0.1169],  # joint4
    [-2.8763, 2.8763],   # joint5
    [0.4398, 4.6216],    # joint6
    [-3.0508, 3.0508],   # joint7
])

# Default home position (joints at center of limits)
FR3_HOME_JOINTS = np.array([0, -0.785, 0, -2.356, 0, 1.571, 0.785])


def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """Create 3x3 rotation matrix from roll, pitch, yaw (XYZ intrinsic)."""
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)

    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])

    return Rz @ Ry @ Rx


def _rotation_z(angle: float) -> np.ndarray:
    """Rotation matrix around Z axis."""
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def _make_transform(xyz, rpy) -> np.ndarray:
    """Create 4x4 homogeneous transform from xyz + rpy."""
    T = np.eye(4)
    T[:3, :3] = _rotation_matrix(*rpy)
    T[:3, 3] = xyz
    return T


def fr3_forward_kinematics(
    joint_angles: np.ndarray,
    base_transform: np.ndarray | None = None,
) -> Dict[str, np.ndarray]:
    """Compute all link transforms for FR3.

    Args:
        joint_angles: 7 joint angles in radians
        base_transform: optional 4x4 base transform (e.g., for arm offset)

    Returns:
        Dict mapping link name to 4x4 world-frame transform.
        Keys: 'link0', 'link1', ..., 'link7', 'hand' (flange/EE)
    """
    assert len(joint_angles) == 7, f"Expected 7 joints, got {len(joint_angles)}"

    transforms = {}

    # Base frame
    T = base_transform.copy() if base_transform is not None else np.eye(4)
    transforms["link0"] = T.copy()

    # Chain through 7 joints + flange
    for i in range(8):
        params = _FR3_JOINT_PARAMS[i]

        # Fixed transform (from URDF joint origin)
        T_fixed = _make_transform(params["xyz"], params["rpy"])
        T = T @ T_fixed

        # Revolute joint rotation (around Z axis)
        if i < 7:
            T_joint = np.eye(4)
            T_joint[:3, :3] = _rotation_z(joint_angles[i])
            T = T @ T_joint

        # Store link transform
        if i < 7:
            transforms[f"link{i + 1}"] = T.copy()
        else:
            transforms["hand"] = T.copy()

    return transforms


def fr3_ee_pose(
    joint_angles: np.ndarray,
    base_transform: np.ndarray | None = None,
) -> np.ndarray:
    """Get end-effector 4x4 transform for given joint angles."""
    transforms = fr3_forward_kinematics(joint_angles, base_transform)
    return transforms["hand"]


def check_joint_limits(joint_angles: np.ndarray) -> bool:
    """Check if joint angles are within FR3 limits."""
    for i in range(7):
        if joint_angles[i] < FR3_JOINT_LIMITS[i, 0] or \
           joint_angles[i] > FR3_JOINT_LIMITS[i, 1]:
            return False
    return True


def clamp_to_joint_limits(joint_angles: np.ndarray) -> np.ndarray:
    """Clamp joint angles to FR3 limits."""
    clamped = joint_angles.copy()
    for i in range(7):
        clamped[i] = np.clip(
            clamped[i],
            FR3_JOINT_LIMITS[i, 0],
            FR3_JOINT_LIMITS[i, 1],
        )
    return clamped
