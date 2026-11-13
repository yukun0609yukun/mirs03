from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Stage 1: 足回り単体動作試験用の最小構成launch。
# Web UI・crowd_monitor・behavior_director・LiDAR・Nav2/SLAMは一切起動しない。
# micro-ROS Agent + wheel_odom + Stage1専用ダッシュボードのみを立ち上げ、
# /cmd_vel・/wheel_ticks・/odom・/estop_state の疎通だけを確認する。


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    default_params = str(bringup_dir / "config" / "nokosahen.yaml")

    params_file = LaunchConfiguration("params_file")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    esp32_port = LaunchConfiguration("esp32_port")
    use_dashboard = LaunchConfiguration("use_dashboard")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
            DeclareLaunchArgument("esp32_port", default_value="/dev/esp32"),
            DeclareLaunchArgument("use_dashboard", default_value="true"),
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "micro_ros_agent",
                    "micro_ros_agent",
                    "serial",
                    "--dev",
                    esp32_port,
                    "-b",
                    "115200",
                ],
                output="screen",
                condition=IfCondition(start_micro_ros_agent),
            ),
            Node(
                package="nokosahen_core",
                executable="wheel_odom",
                name="wheel_odom",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nokosahen_core",
                executable="stage1_dashboard",
                name="stage1_dashboard",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_dashboard),
            ),
        ]
    )
