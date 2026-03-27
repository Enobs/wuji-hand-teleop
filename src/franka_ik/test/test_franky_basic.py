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


def cmd_sine(robot: Robot, speed: float):
    """Sine motion on joint 4 (elbow), +-10 deg, 3 cycles."""
    import math

    q = list(robot.current_joint_positions)
    j4_orig = q[3]
    amplitude = np.radians(10)
    freq = 0.3  # Hz
    duration = 3.0 / freq  # 3 full cycles
    dt = 0.05  # 50ms between waypoints

    print(f"Sine on joint 4: {np.degrees(j4_orig):.1f} deg +- 10 deg")
    print(f"Frequency: {freq} Hz, Duration: {duration:.1f}s")
    print(f"Speed: {speed*100:.0f}%")
    input("Press Enter to start (Ctrl+C to cancel)...")

    robot.relative_dynamics_factor = speed
    t0 = time.time()

    try:
        while True:
            t = time.time() - t0
            if t > duration:
                break
            q[3] = j4_orig + amplitude * math.sin(2 * math.pi * freq * t)
            motion = JointWaypointMotion([JointWaypoint(q)])
            robot.move(motion, asynchronous=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass

    # Return to original
    q[3] = j4_orig
    motion = JointWaypointMotion([JointWaypoint(q)])
    robot.move(motion)
    print("Sine done, returned to original position.")


def cmd_circle(robot: Robot, speed: float):
    """Circle in Cartesian space using joint-level IK approximation.

    Moves joints 2,4 in coordinated sine/cosine to trace a circle
    in the EE XZ plane. Radius ~5cm.
    """
    import math

    q = list(robot.current_joint_positions)
    j2_orig = q[1]
    j4_orig = q[3]
    # Small joint amplitudes → ~5cm Cartesian radius
    amp2 = np.radians(5)
    amp4 = np.radians(5)
    freq = 0.2  # Hz
    duration = 200.0 / freq  # 2 full cycles
    dt = 0.05

    print(f"Circle: joints 2,4 coordinated, ~5cm radius")
    print(f"Frequency: {freq} Hz, Duration: {duration:.1f}s")
    print(f"Speed: {speed*100:.0f}%")
    input("Press Enter to start (Ctrl+C to cancel)...")

    robot.relative_dynamics_factor = speed
    t0 = time.time()

    try:
        while True:
            t = time.time() - t0
            if t > duration:
                break
            phase = 2 * math.pi * freq * t
            q[1] = j2_orig + amp2 * math.sin(phase)
            q[3] = j4_orig + amp4 * math.cos(phase)
            motion = JointWaypointMotion([JointWaypoint(q)])
            robot.move(motion, asynchronous=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass

    # Return to original
    q[1] = j2_orig
    q[3] = j4_orig
    motion = JointWaypointMotion([JointWaypoint(q)])
    robot.move(motion)
    print("Circle done, returned to original position.")


def main():
    parser = argparse.ArgumentParser(description="Basic Franka FR3 test")
    parser.add_argument("--ip", required=True, help="Robot IP (e.g. 192.168.1.1)")
    parser.add_argument("--speed", type=float, default=0.1,
                        help="Dynamic factor 0.0-1.0 (default 0.1 = 10%%)")
    parser.add_argument("command", choices=["state", "home", "wiggle", "sine", "circle", "recover"],
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
    elif args.command == "sine":
        cmd_sine(robot, args.speed)
    elif args.command == "circle":
        cmd_circle(robot, args.speed)


if __name__ == "__main__":
    main()
