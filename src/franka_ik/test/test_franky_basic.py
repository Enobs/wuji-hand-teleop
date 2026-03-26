#!/usr/bin/env python3
"""
Basic Franka FR3 control test via franky.

Usage:
  # Read current state
  python3 test_franky_basic.py --ip 192.168.1.1 state

  # Move to home
  python3 test_franky_basic.py --ip 192.168.1.1 home

  # Small joint wiggle (joint 7 +-5 deg)
  python3 test_franky_basic.py --ip 192.168.1.1 wiggle

  # Sine motion on joint 4 (+-10 deg, 3 cycles)
  python3 test_franky_basic.py --ip 192.168.1.1 sine

  # Circle in Cartesian XZ plane (5cm radius, 2 cycles)
  python3 test_franky_basic.py --ip 192.168.1.1 circle

  # Recover from errors
  python3 test_franky_basic.py --ip 192.168.1.1 recover
"""
import argparse
import sys
import time
import numpy as np

try:
    from franky import (
        Robot,
        JointWaypointMotion,
        JointWaypoint,
        ReferenceType,
    )
except ImportError:
    print("ERROR: pip install franky-panda")
    sys.exit(1)


FR3_HOME = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]


def cmd_state(robot: Robot):
    """Print current robot state."""
    state = robot.current_joint_positions
    print(f"Joint positions: {[round(x, 4) for x in state]}")
    print(f"Home:            {FR3_HOME}")
    diff = np.array(state) - np.array(FR3_HOME)
    print(f"Diff from home:  {[round(x, 4) for x in diff]}")
    print(f"Max diff:        {max(abs(diff)):.4f} rad ({np.degrees(max(abs(diff))):.1f} deg)")


def cmd_recover(robot: Robot):
    """Recover from errors."""
    print("Recovering from errors...")
    robot.recover_from_errors()
    print("Done.")


def cmd_home(robot: Robot, speed: float):
    """Move to home position."""
    print(f"Current: {[round(x, 4) for x in robot.current_joint_positions]}")
    print(f"Target:  {FR3_HOME}")
    print(f"Speed:   {speed*100:.0f}%")

    input("Press Enter to move to home (Ctrl+C to cancel)...")

    robot.relative_dynamics_factor = speed
    motion = JointWaypointMotion([JointWaypoint(FR3_HOME)])
    robot.move(motion)
    print("Home reached.")
    cmd_state(robot)


def cmd_wiggle(robot: Robot, speed: float):
    """Small wiggle on joint 7 to verify control works."""
    q = list(robot.current_joint_positions)
    q7_orig = q[6]
    delta = np.radians(5)  # 5 degrees

    print(f"Wiggling joint 7: {np.degrees(q7_orig):.1f} deg +- 5 deg")
    print(f"Speed: {speed*100:.0f}%")
    input("Press Enter to start (Ctrl+C to cancel)...")

    robot.relative_dynamics_factor = speed

    for i in range(3):
        # Forward
        q[6] = q7_orig + delta
        motion = JointWaypointMotion([JointWaypoint(q)])
        robot.move(motion)
        print(f"  [{i+1}/3] +5 deg")

        # Back
        q[6] = q7_orig - delta
        motion = JointWaypointMotion([JointWaypoint(q)])
        robot.move(motion)
        print(f"  [{i+1}/3] -5 deg")

    # Return to original
    q[6] = q7_orig
    motion = JointWaypointMotion([JointWaypoint(q)])
    robot.move(motion)
    print("Wiggle done, returned to original position.")


def main():
    parser = argparse.ArgumentParser(description="Basic Franka FR3 test")
    parser.add_argument("--ip", required=True, help="Robot IP (e.g. 192.168.1.1)")
    parser.add_argument("--speed", type=float, default=0.1,
                        help="Dynamic factor 0.0-1.0 (default 0.1 = 10%%)")
    parser.add_argument("command", choices=["state", "home", "wiggle", "recover"],
                        help="Command to run")
    args = parser.parse_args()

    print(f"Connecting to FR3 at {args.ip}...")
    from franky import RealtimeConfig
    robot = Robot(args.ip, realtime_config=RealtimeConfig.Ignore)
    print(f"Connected. Joint pos: {[round(x,3) for x in robot.current_joint_positions]}")

    if args.command == "state":
        cmd_state(robot)
    elif args.command == "recover":
        cmd_recover(robot)
    elif args.command == "home":
        cmd_home(robot, args.speed)
    elif args.command == "wiggle":
        cmd_wiggle(robot, args.speed)


if __name__ == "__main__":
    main()
