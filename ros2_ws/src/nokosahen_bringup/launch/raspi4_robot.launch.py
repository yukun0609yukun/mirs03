from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


import os

# 【2026-10-21】旧構成の起動ファイル(全体レビューの指摘)。いまの運用の起動(scripts/start_all.sh、run_persistent_base_supervised.sh、
# ダッシュボードからの地図作成・位置合わせ=mapping_mode / localization_mode)とは別物で、次が入っていない:
#   - Nav2の設定の書き換え(最高速度・安全距離。nav_speed.write_rewritten_params)と、BTファイル名の絶対パス化(読み込めず、Nav2が起動に失敗する)
#   - Nav2の goal_pose の付け替え(bt_navigatorがnav2_goal_routerを素通りして、緊急停止・アンケート・範囲の確認なしに走り出す)
# そのため、既定では起動しない。どうしても使う(Gazeboや旧構成の確認)ときは、NOKOSAHEN_ALLOW_LEGACY_LAUNCH=1 を付ける。
LEGACY_LAUNCH_MESSAGE = (
    "{name} は旧構成の起動ファイルのため、起動しません。いまの起動は bash ~/mirs2603_nokosahen/scripts/start_all.sh <manual|full|nav_test> を使ってください"
    "(docs/startup.md)。どうしても使うときは、環境変数 NOKOSAHEN_ALLOW_LEGACY_LAUNCH=1 を付けて実行してください"
    "(ただし、Nav2の安全設定・目的地の確認が入っていない)。"
)


def _legacy_launch_guard(name):
    if os.environ.get("NOKOSAHEN_ALLOW_LEGACY_LAUNCH", "") != "1":
        raise RuntimeError(LEGACY_LAUNCH_MESSAGE.format(name=name))


def generate_launch_description():
    _legacy_launch_guard("raspi4_robot.launch.py")
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_camera_counter = LaunchConfiguration("use_camera_counter")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    esp32_port = LaunchConfiguration("esp32_port")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    use_person_target_detector = LaunchConfiguration("use_person_target_detector")
    use_voice_node = LaunchConfiguration("use_voice_node")
    use_stage2_dashboard = LaunchConfiguration("use_stage2_dashboard")
    use_full_dashboard = LaunchConfiguration("use_full_dashboard")
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
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=str(bringup_dir / "config" / "nav2_params_raspi4.yaml"),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=str(
                    bringup_dir / "config" / "slam_toolbox_online_async.yaml"
                ),
            ),
            DeclareLaunchArgument("use_slam", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("use_camera_counter", default_value="false"),
            DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
            DeclareLaunchArgument("esp32_port", default_value="/dev/esp32"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            # RPLIDAR S1(360度)固有の値。A1/A2/A3とはボーレート・スキャン仕様が異なる。
            DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
            DeclareLaunchArgument("lidar_baudrate", default_value="256000"),
            DeclareLaunchArgument("use_person_target_detector", default_value="true"),
            DeclareLaunchArgument("use_voice_node", default_value="true"),
            DeclareLaunchArgument("use_stage2_dashboard", default_value="false"),
            DeclareLaunchArgument("use_full_dashboard", default_value="false"),
            # docs/mapping_and_calibration.md の較正結果をここへ反映する
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.15"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            Node(
                # 【重要】aptの`ros-jazzy-rplidar-ros`(2.1.0)の
                # 'rplidar_composition'はRPLIDAR S1でSEGVクラッシュするバグを
                # 確認した(2026-07-15、Jetson実機)。Slamtec公式`ros2`ブランチを
                # ソースビルドした'rplidar_node'を使うこと。
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
                condition=IfCondition(use_lidar),
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
                    "use_camera_counter": use_camera_counter,
                    "use_person_target_detector": use_person_target_detector,
                    "use_voice_node": use_voice_node,
                    "use_stage2_dashboard": use_stage2_dashboard,
                    "use_full_dashboard": use_full_dashboard,
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
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("slam_toolbox"),
                            "launch",
                            "online_async_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "slam_params_file": slam_params_file,
                    "use_sim_time": use_sim_time,
                }.items(),
                condition=IfCondition(use_slam),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [
                            FindPackageShare("nav2_bringup"),
                            "launch",
                            "navigation_launch.py",
                        ]
                    )
                ),
                launch_arguments={
                    "params_file": nav2_params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                }.items(),
                condition=IfCondition(use_nav2),
            ),
            # 【2026-10-26】CPU削減: /tf(約70Hz)を直接購読していた自作ノード(nav2_goal_router/wall_escape/motion_arbiter)の代わりに、
            # このノード1つが map→base_link / map→odom を5Hzで /nokosahen/robot_pose へ配る(pose_relay.py)。この3ノードより先に置く。
            Node(
                package="nokosahen_core",
                executable="pose_relay",
                name="pose_relay",
                output="screen",
                condition=IfCondition(use_nav2),
            ),
            Node(
                package="nokosahen_core",
                executable="nav2_goal_router",
                name="nav2_goal_router",
                output="screen",
                condition=IfCondition(use_nav2),
            ),
        ]
    )
