"""
Franka Dual-Arm + PICO Visualization

启动内容:
  1. robot_state_publisher  — 发布 fr3_dual URDF + TF
  2. pico_input_node        — PICO 追踪 → /left_arm_target_pose, /right_arm_target_pose
  3. franka_ik_node         — pink IK → /joint_states
  4. rviz2                  — 可视化

使用方法:
  # 真实 PICO (live)
  ros2 launch franka_ik franka_pico_viz.launch.py

  # 录制回放 (无需硬件)
  ros2 launch franka_ik franka_pico_viz.launch.py data_source_type:=recorded
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _pkg(name: str) -> Path:
    return Path(get_package_share_directory(name))


def generate_launch_description():
    urdf_path = _pkg("franka_description") / "robots" / "fr3" / "fr3_dual.urdf"
    rviz_config = _pkg("franka_ik") / "rviz" / "franka_pico.rviz"
    robot_description = urdf_path.read_text()

    # ---------- launch args ----------
    data_source_arg = DeclareLaunchArgument(
        "data_source_type", default_value="live",
        description="'live' (real PICO) | 'recorded' (replay) | 'webxr' (PICO WebXR) | 'none' (no input)")
    enable_input_arg = DeclareLaunchArgument(
        "enable_input", default_value="true",
        description="Set 'false' to disable pico input node (for testing)")
    recorded_file_arg = DeclareLaunchArgument(
        "recorded_file_path",
        default_value=str(
            Path(get_package_share_directory("pico_input")).parent.parent.parent.parent
            / "src" / "input_devices" / "pico_input" / "record"
            / "trackingData_sample_static.txt"),
        description="Path to recorded tracking data")
    playback_speed_arg = DeclareLaunchArgument(
        "playback_speed", default_value="1.0",
        description="Playback speed multiplier")

    # ---------- nodes ----------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    pico_input_node = Node(
        package="franka_ik",
        executable="franka_pico_input_node",
        name="franka_pico_input_node",
        condition=IfCondition(LaunchConfiguration("enable_input")),
        output="screen",
        emulate_tty=True,
        parameters=[{
            "data_source_type":   LaunchConfiguration("data_source_type"),
            "recorded_file_path": LaunchConfiguration("recorded_file_path"),
            "playback_speed":     LaunchConfiguration("playback_speed"),
            "auto_init_delay":    3.0,
            "publish_rate":       90.0,
            "left_init_pos":      [0.307,  0.3, 0.590],
            "right_init_pos":     [0.307, -0.3, 0.590],
            "tracker_serial_left":  "190058G",
            "tracker_serial_right": "190600G",
        }],
    )

    franka_ik_node = Node(
        package="franka_ik",
        executable="franka_ik_node",
        name="franka_ik_node",
        output="screen",
        parameters=[{
            "rate_hz": 100.0,
            "dt": 0.01,
            "pos_cost": 1.0,
            "ori_cost": 0.01,
            "solver": "quadprog",
        }],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(rviz_config)],
        output="screen",
    )

    return LaunchDescription([
        data_source_arg,
        enable_input_arg,
        recorded_file_arg,
        playback_speed_arg,
        robot_state_publisher,
        pico_input_node,
        franka_ik_node,
        rviz_node,
    ])
