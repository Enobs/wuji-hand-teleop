"""
Franka PICO Real Robot Control (with RViz)

PICO → franka_pico_input → franka_ik (Pink + FCL) → /joint_states → RViz
                                                   → franka_output → 真机

IK 已包含关节限位 + FCL 碰撞检测，RViz 和真机用同一组关节角。
真机通过 franky Ruckig 平滑跟随，延迟极小。

Usage:
  # 右臂 (5% 速度)
  ros2 launch franka_ik franka_pico_real.launch.py right_ip:=172.16.0.2 enable_left:=false

  # 双臂
  ros2 launch franka_ik franka_pico_real.launch.py left_ip:=172.16.0.1 right_ip:=172.16.0.2

  # 右臂, 录制回放
  ros2 launch franka_ik franka_pico_real.launch.py right_ip:=172.16.0.2 enable_left:=false data_source_type:=recorded
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

    # Robot connection args
    left_ip_arg = DeclareLaunchArgument(
        "left_ip", default_value="192.168.1.1",
        description="Left arm FCI IP")
    right_ip_arg = DeclareLaunchArgument(
        "right_ip", default_value="192.168.2.1",
        description="Right arm FCI IP")
    enable_left_arg = DeclareLaunchArgument(
        "enable_left", default_value="true",
        description="Enable left arm")
    enable_right_arg = DeclareLaunchArgument(
        "enable_right", default_value="true",
        description="Enable right arm")
    dynamic_rel_arg = DeclareLaunchArgument(
        "dynamic_rel", default_value="0.05",
        description="Ruckig speed factor 0.0-1.0 (default 5%)")

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

    # NOTE: franka_output_node 也单独启动:
    #   ROS_DOMAIN_ID=42 python3 -m franka_ik.franka_output_node \
    #       --ros-args -p right_ip:="172.16.0.2" -p enable_left:=false -p dynamic_rel:=0.05

    # Output: subscribes to /joint_states, sends to real robot via franky
    # (保留定义但不在 LaunchDescription 中启动，供参考)
    franka_output_node = Node(
        package="franka_ik",
        executable="franka_output_node",
        name="franka_output_node",
        output="screen",
        parameters=[{
            "left_ip":      LaunchConfiguration("left_ip"),
            "right_ip":     LaunchConfiguration("right_ip"),
            "enable_left":  LaunchConfiguration("enable_left"),
            "enable_right": LaunchConfiguration("enable_right"),
            "dynamic_rel":  LaunchConfiguration("dynamic_rel"),
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
        recorded_file_arg,
        playback_speed_arg,
        left_ip_arg,
        right_ip_arg,
        enable_left_arg,
        enable_right_arg,
        dynamic_rel_arg,
        robot_state_publisher,
        pico_input_node,
        rviz_node,
    ])
