from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# メニュー毎の決済場所ユニット(Webカメラ+Raspberry Pi 4)用launch。
# 全ユニットに同一のコード・このlaunchファイルをそのまま導入すればよい
# (camera_idは実行時にCPUシリアルから自動で決まるため、機体ごとの
# 書き換えは不要)。メニュー割り当ては起動後にJetson側の完成版
# ダッシュボードから行う。
#
# 使い方:
#   ros2 launch nokosahen_checkout checkout_camera.launch.py


def generate_launch_description():
    pkg_dir = Path(get_package_share_directory("nokosahen_checkout"))
    default_params = str(pkg_dir / "config" / "checkout_cam.yaml")
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="nokosahen_checkout",
                executable="checkout_camera",
                name="checkout_camera_node",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
