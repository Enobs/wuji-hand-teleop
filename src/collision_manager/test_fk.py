#!/usr/bin/env python3
"""Quick test: FR3 FK sanity check.

Run: python3 src/collision_manager/test_fk.py
"""
import numpy as np
from collision_manager.fr3_kinematics import (
    fr3_forward_kinematics,
    fr3_ee_pose,
    FR3_HOME_JOINTS,
    FR3_JOINT_LIMITS,
    check_joint_limits,
)


def test_home_position():
    """Test FK at home position."""
    print("=== FR3 FK Test ===\n")
    print(f"Home joints (rad): {FR3_HOME_JOINTS}")
    print(f"Home joints (deg): {np.degrees(FR3_HOME_JOINTS)}")
    print()

    transforms = fr3_forward_kinematics(FR3_HOME_JOINTS)

    print("Link transforms (position only):")
    for name, T in transforms.items():
        pos = T[:3, 3]
        print(f"  {name:6s}: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}")

    ee = transforms["hand"]
    print(f"\nEE position: {ee[:3, 3]}")
    print(f"EE rotation:\n{ee[:3, :3]}")
    print()


def test_zero_position():
    """Test FK at zero position (all joints = 0)."""
    q = np.zeros(7)
    print("=== Zero position ===")
    print(f"Joints: {q}")

    transforms = fr3_forward_kinematics(q)
    print("Link positions:")
    for name, T in transforms.items():
        pos = T[:3, 3]
        print(f"  {name:6s}: x={pos[0]:+.4f}  y={pos[1]:+.4f}  z={pos[2]:+.4f}")
    print()


def test_dual_arm():
    """Test dual arm with offsets."""
    print("=== Dual arm (60mm spacing) ===")
    left_base = np.eye(4)
    left_base[1, 3] = 0.03  # +30mm Y

    right_base = np.eye(4)
    right_base[1, 3] = -0.03  # -30mm Y

    q = FR3_HOME_JOINTS

    left_transforms = fr3_forward_kinematics(q, left_base)
    right_transforms = fr3_forward_kinematics(q, right_base)

    left_ee = left_transforms["hand"][:3, 3]
    right_ee = right_transforms["hand"][:3, 3]

    print(f"Left  EE: {left_ee}")
    print(f"Right EE: {right_ee}")
    print(f"Distance: {np.linalg.norm(left_ee - right_ee):.4f}m")
    print()


def test_joint_limits():
    """Test joint limit checking."""
    print("=== Joint limits ===")
    print(f"Home in limits: {check_joint_limits(FR3_HOME_JOINTS)}")
    bad_joints = np.array([5, 0, 0, 0, 0, 0, 0])
    print(f"Bad joints in limits: {check_joint_limits(bad_joints)}")
    print()


if __name__ == "__main__":
    test_home_position()
    test_zero_position()
    test_dual_arm()
    test_joint_limits()
    print("All tests passed!")
