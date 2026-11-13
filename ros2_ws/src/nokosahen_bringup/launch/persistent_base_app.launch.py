from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 【2026-08-05追加】Jetson 2台分散構成の「アプリ・表示系(Jetson B)」側。
# 対になる persistent_base_control.launch.py の冒頭コメント参照(分散化の
# 経緯・前提条件はそちら)。
#
# ここに置くノードはいずれもHTTP経由で動く、あるいはTFの厳密なタイミングに
# 依存しないものだけを選んでいる(足回り制御・LiDAR・Nav2・SLAM・人物検知・
# 近接安全停止はJetson A側に残す)。crowd_monitorは/scanを直接購読するため
# LiDARスキャンがネットワーク越しに届く前提だが、混雑判定は多少の遅延が
# あっても実用上問題にならないためこちらに置いている。
#
# 使い方(Jetson B側):
#   source /opt/ros/jazzy/setup.bash
#   source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash
#   export ROS_DOMAIN_ID=77
#   ros2 launch nokosahen_bringup persistent_base_app.launch.py
#
# 【未検証】このファイルは2台目Jetsonのセットアップ前に用意した準備段階の
# ものであり、実機2台構成での動作確認はまだ行っていない。


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    params_file = LaunchConfiguration("params_file")
    use_camera_counter = LaunchConfiguration("use_camera_counter")
    use_voice_node = LaunchConfiguration("use_voice_node")
    use_stage2_dashboard = LaunchConfiguration("use_stage2_dashboard")
    use_full_dashboard = LaunchConfiguration("use_full_dashboard")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(bringup_dir / "config" / "nokosahen.yaml"),
            ),
            DeclareLaunchArgument("use_camera_counter", default_value="false"),
            DeclareLaunchArgument("use_voice_node", default_value="true"),
            # Stage2用(残数カウント無効)とcompleteな完成版(残数カウント有効)は
            # 同時にtrueにしないこと(どちらもポート8090を使う想定)。
            DeclareLaunchArgument("use_stage2_dashboard", default_value="true"),
            DeclareLaunchArgument("use_full_dashboard", default_value="false"),
            Node(
                package="nokosahen_core",
                executable="inventory_manager",
                name="inventory_manager",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nokosahen_core",
                executable="web_ui",
                name="web_ui",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nokosahen_core",
                executable="menu_schedule",
                name="menu_schedule",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nokosahen_core",
                executable="crowd_monitor",
                name="crowd_monitor",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="nokosahen_core",
                executable="camera_counter",
                name="camera_counter",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_camera_counter),
            ),
            Node(
                package="nokosahen_voice",
                executable="voice_node",
                name="voice_node",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_voice_node),
            ),
            # 【2026-10-13】状況アナウンス(「探索中です」等、docs/announcements.md)。設定ファイルが無い間は何も鳴らさない。
            Node(
                package="nokosahen_core",
                executable="announce_node",
                name="announce_node",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_voice_node),
            ),
            Node(
                package="nokosahen_core",
                executable="stage2_dashboard",
                name="stage2_dashboard",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_stage2_dashboard),
            ),
            Node(
                package="nokosahen_core",
                executable="full_dashboard",
                name="full_dashboard",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_full_dashboard),
            ),
        ]
    )
