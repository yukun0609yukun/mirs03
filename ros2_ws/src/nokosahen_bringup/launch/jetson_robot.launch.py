from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


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
    _legacy_launch_guard("jetson_robot.launch.py")
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    raspi_launch = bringup_dir / "launch" / "raspi4_robot.launch.py"
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

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(bringup_dir / "config" / "nokosahen.yaml"),
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=str(bringup_dir / "config" / "nav2_params_jetson.yaml"),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=str(
                    bringup_dir / "config" / "slam_toolbox_online_async.yaml"
                ),
            ),
            DeclareLaunchArgument("use_slam", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            # Stage 2(残数カウントは一旦除外)に合わせてデフォルトfalse。
            # 残数カウントを使う段になったら true で起動する。
            DeclareLaunchArgument("use_camera_counter", default_value="false"),
            DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
            DeclareLaunchArgument("esp32_port", default_value="/dev/esp32"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
            DeclareLaunchArgument("lidar_baudrate", default_value="256000"),
            DeclareLaunchArgument("use_person_target_detector", default_value="true"),
            DeclareLaunchArgument("use_voice_node", default_value="true"),
            # Stage2の監視・操作ダッシュボード(WASD運転・地図クリック目的地・
            # ソフト緊急停止)。完成版(残数カウントあり)は
            # jetson_robot_full.launch.py 側でfull_dashboardを使う。
            DeclareLaunchArgument("use_stage2_dashboard", default_value="true"),
            DeclareLaunchArgument("use_full_dashboard", default_value="false"),
            # docs/mapping_and_calibration.md の較正結果をここへ反映する
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.15"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(raspi_launch)),
                launch_arguments={
                    "params_file": params_file,
                    "nav2_params_file": nav2_params_file,
                    "slam_params_file": slam_params_file,
                    "use_slam": use_slam,
                    "use_nav2": use_nav2,
                    "use_camera_counter": use_camera_counter,
                    "start_micro_ros_agent": start_micro_ros_agent,
                    "esp32_port": esp32_port,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "use_lidar": use_lidar,
                    "lidar_port": lidar_port,
                    "lidar_baudrate": lidar_baudrate,
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
        ]
    )
