"""
Franka PICO Simulation (RViz only, no real robot)

PICO → franka_pico_input → franka_ik → /joint_states → RViz

Usage:
  # 真实 PICO WebXR
  ros2 launch franka_ik franka_pico_sim.launch.py

  # 录制回放
  ros2 launch franka_ik franka_pico_sim.launch.py data_source_type:=recorded
"""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
        "data_source_type", default_value="webxr",
        description="'live' | 'recorded' | 'webxr' | 'none'")
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

    # NOTE: franka_ik_node 需要单独启动 (numpy 兼容问题):
    #   ROS_DOMAIN_ID=42 python3 -m franka_ik.franka_ik_node

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", str(rviz_config)],
        output="screen",
    )

    return LaunchDescription([
        data_source_arg,
        recorded_file_arg,
        playback_speed_arg,
        robot_state_publisher,
        pico_input_node,
        rviz_node,
    ])
