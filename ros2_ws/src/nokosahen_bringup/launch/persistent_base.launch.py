from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# ダッシュボードから地図の作成・停止・リセット・読み込みをボタン操作できるように
# するため、常時起動しておく「土台」(ESP32・LiDAR・カメラ・ダッシュボード・
# behavior_director等)と、切り替えて使う「地図モード」(mapping_mode.launch.py /
# localization_mode.launch.py)を分離した。
#
# 今までの jetson_robot.launch.py / raspi4_robot.launch.py はslam_toolbox+nav2を
# 込みで常時起動する構成のままなので、CLIから昔ながらの手順で使いたい場合は
# 引き続きそちらを使ってよい(削除していない)。ダッシュボードから地図操作を
# したい場合はこちらを起動してから、地図セクションの「開始/停止/リセット/読み込み」
# ボタンでmapping_mode/localization_modeを切り替える。
#
# 使い方:
#   ros2 launch nokosahen_bringup persistent_base.launch.py \
#       esp32_port:=/dev/esp32 lidar_port:=/dev/rplidar


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    params_file = LaunchConfiguration("params_file")
    use_behavior_director = LaunchConfiguration("use_behavior_director")
    use_camera_counter = LaunchConfiguration("use_camera_counter")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    esp32_port = LaunchConfiguration("esp32_port")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    use_person_target_detector = LaunchConfiguration("use_person_target_detector")
    use_rear_cameras = LaunchConfiguration("use_rear_cameras")
    use_voice_node = LaunchConfiguration("use_voice_node")
    use_stage2_dashboard = LaunchConfiguration("use_stage2_dashboard")
    use_full_dashboard = LaunchConfiguration("use_full_dashboard")
    use_survey_monitor = LaunchConfiguration("use_survey_monitor")
    use_device_watchdog = LaunchConfiguration("use_device_watchdog")
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    core_launch = bringup_dir / "launch" / "core.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(bringup_dir / "config" / "nokosahen.yaml"),
            ),
            DeclareLaunchArgument("use_camera_counter", default_value="false"),
            DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
            DeclareLaunchArgument("esp32_port", default_value="/dev/esp32"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            # RPLIDAR S1(360度)固有の値。A1/A2/A3とはボーレート・スキャン仕様が異なる。
            DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
            DeclareLaunchArgument("lidar_baudrate", default_value="256000"),
            DeclareLaunchArgument("use_person_target_detector", default_value="true"),
            DeclareLaunchArgument("use_rear_cameras", default_value="true"),  # 後ろ向きカメラ(2026-10-22)。無い構成は false
            DeclareLaunchArgument("use_voice_node", default_value="true"),
            # 【2026-07-22追加】ラジコン運用のみの階層と自律行動ありの階層を
            # 切り替える。scripts/start_persistent_base_rc_only.sh はこれを
            # falseにして呼ぶ(scripts/start_persistent_base.shは既定のtrueのまま)。
            DeclareLaunchArgument("use_behavior_director", default_value="true"),
            DeclareLaunchArgument("use_stage2_dashboard", default_value="true"),
            DeclareLaunchArgument("use_full_dashboard", default_value="false"),
            DeclareLaunchArgument("use_survey_monitor", default_value="true"),
            # 【2026-11-06】機器監視(docs/device_watchdog.md)。trueのとき、rplidar_nodeとmicro_ros_agentはwatchdogが起動・停止・再起動する
            # (launchは起動しない=二重起動と、launchの管理とのぶつかりを避ける)。falseなら従来どおりlaunchが起動する。
            DeclareLaunchArgument("use_device_watchdog", default_value="true"),
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.15"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            Node(
                # 【重要】aptの`ros-jazzy-rplidar-ros`(2.1.0)の'rplidar_composition'は
                # RPLIDAR S1でSEGVクラッシュするバグを確認した(2026-07-15、Jetson実機)。
                # Slamtec公式`ros2`ブランチをソースビルドした'rplidar_node'を使うこと。
                # scripts/install_rplidar_from_source.sh 参照。
                package="rplidar_ros",
                executable="rplidar_node",
                name="rplidar_node",
                output="screen",
                # 2026-10-05: 生スキャンは/scan_rawへ。自己反射(四隅の支柱)を除去した
                # /scanはscan_self_filterが配信する(nokosahen.yaml参照)。
                remappings=[("scan", "scan_raw")],
                parameters=[{
                    "channel_type": "serial",
                    "serial_port": lidar_port,
                    "serial_baudrate": lidar_baudrate,
                    "frame_id": "laser",
                    "inverted": False,
                    "angle_compensate": True,
                }],
                # watchdogが管理するとき(既定)は起動しない
                condition=IfCondition(PythonExpression(["'", use_lidar, "' == 'true' and '", use_device_watchdog, "' != 'true'"])),
            ),
            Node(
                package="nokosahen_core",
                executable="scan_self_filter",
                name="scan_self_filter",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_lidar),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(core_launch)),
                launch_arguments={
                    "params_file": params_file,
                    "use_mock": "false",
                    "use_behavior_director": use_behavior_director,
                    "use_camera_counter": use_camera_counter,
                    "use_person_target_detector": use_person_target_detector,
                    "use_rear_cameras": use_rear_cameras,
                    "use_voice_node": use_voice_node,
                    "use_stage2_dashboard": use_stage2_dashboard,
                    "use_full_dashboard": use_full_dashboard,
                    "use_survey_monitor": use_survey_monitor,
                    "camera_x": camera_x,
                    "camera_y": camera_y,
                    "camera_z": camera_z,
                    "camera_roll": camera_roll,
                    "camera_pitch": camera_pitch,
                    "camera_yaw": camera_yaw,
                }.items(),
            ),
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "micro_ros_agent", "micro_ros_agent",
                    "serial", "--dev", esp32_port, "-b", "115200",
                ],
                output="screen",
                condition=IfCondition(PythonExpression(["'", start_micro_ros_agent, "' == 'true' and '", use_device_watchdog, "' != 'true'"])),
            ),
            # 機器監視。rplidar_node / micro_ros_agent をこのノードが起動・停止・再起動する(上の2つはlaunchからは起動しない)。
            # respawn: watchdog自身が落ちても戻す(戻ったとき、動いている子プロセスは引き継ぐ。二重起動しない)。
            Node(
                package="nokosahen_core",
                executable="device_watchdog",
                name="device_watchdog",
                output="screen",
                respawn=True,
                respawn_delay=3.0,
                parameters=[params_file, {
                    "lidar.enabled": use_lidar,
                    "lidar.paths": lidar_port,
                    "lidar.baud": lidar_baudrate,
                    "esp32.enabled": start_micro_ros_agent,
                    "esp32.paths": esp32_port,
                }],
                condition=IfCondition(use_device_watchdog),
            ),
        ]
    )
