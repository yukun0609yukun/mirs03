from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 【2026-08-05追加】Jetson 2台分散構成の「足回り・安全系(Jetson A)」側。
#
# 2026-07-24の明電舎PoC現地検証で、1台のJetson Orin Nanoにロボット制御一式
# (ESP32/LiDAR/カメラ)+Nav2+SLAM+YOLO人物検知を同時稼働させたところ
# load average 17〜40台のCPU過負荷が発生し、TF反映が間に合わずslam_toolboxが
# LiDARスキャンを処理しきれない(地図が生成できない)不具合や、WASD操作の
# 応答遅延が発生した(docs/known_issues_and_startup_checklist.md 補足6参照)。
#
# この対策として、TF・安全判定のタイミングがシビアなノード群(足回り制御・
# LiDAR・Nav2・SLAM・人物検知・近接安全停止)をこちら(Jetson A)に集約し、
# HTTP経由で動くアプリ系ノード(ダッシュボード・音声・メニュー管理)は
# persistent_base_app.launch.py としてJetson Bに分離する。
#
# 【前提条件】
#   - Jetson A・B は同じLANに有線(LANケーブル)で接続し、同じROS_DOMAIN_ID
#     (既定77)を設定すること。DDSが自動的にノードを発見するため、特別な
#     「クラスター化」ソフトは不要。
#   - Jetson A・B は同じJetPackイメージ(Ubuntu 24.04 + ROS 2 Jazzy)で
#     揃えること。ROS 2は異なるディストリビューション間の相互運用を公式には
#     保証していない。
#   - mapping_mode.launch.py / localization_mode.launch.py は従来通り
#     Jetson A側でダッシュボードの地図操作ボタンから起動する
#     (Nav2/SLAM/frontier_explorerはこちらに同居するのが前提)。
#
# 使い方(Jetson A側):
#   source /opt/ros/jazzy/setup.bash
#   source ~/lidar_src_build/install/setup.bash
#   source ~/microros_ws/install/setup.bash
#   source ~/mirs2603_nokosahen/ros2_ws/install/setup.bash
#   export ROS_DOMAIN_ID=77
#   ros2 launch nokosahen_bringup persistent_base_control.launch.py \
#       esp32_port:=/dev/esp32 lidar_port:=/dev/rplidar
#
# 【未検証】このファイルは2台目Jetsonのセットアップ前に用意した準備段階の
# ものであり、実機2台構成での動作確認はまだ行っていない。


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    params_file = LaunchConfiguration("params_file")
    use_behavior_director = LaunchConfiguration("use_behavior_director")
    start_micro_ros_agent = LaunchConfiguration("start_micro_ros_agent")
    esp32_port = LaunchConfiguration("esp32_port")
    use_lidar = LaunchConfiguration("use_lidar")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    use_person_target_detector = LaunchConfiguration("use_person_target_detector")
    use_rear_cameras = LaunchConfiguration("use_rear_cameras")
    use_survey_monitor = LaunchConfiguration("use_survey_monitor")
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
            DeclareLaunchArgument("start_micro_ros_agent", default_value="true"),
            DeclareLaunchArgument("esp32_port", default_value="/dev/esp32"),
            DeclareLaunchArgument("use_lidar", default_value="true"),
            # RPLIDAR S1(360度)固有の値。A1/A2/A3とはボーレート・スキャン仕様が異なる。
            DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
            DeclareLaunchArgument("lidar_baudrate", default_value="256000"),
            DeclareLaunchArgument("use_person_target_detector", default_value="true"),
            # 【2026-10-23】後ろ向きカメラ(左後ろ・右後ろ)の人物検出(core.launch.py と同じ。分散構成にも入れ忘れていた)。
            # カメラが無い/外した構成では false(既定 true)。use_person_target_detector も true のときだけ起動する。
            DeclareLaunchArgument("use_rear_cameras", default_value="true"),
            # ラジコン運用オンリーの場合はfalseにする
            # (scripts/start_persistent_base_control_rc_only.sh 参照)。
            DeclareLaunchArgument("use_behavior_director", default_value="true"),
            # 【2026-10-09追加】アンケート回答中にロボットを止める橋渡し(docs/survey_robot_hold.md)。
            DeclareLaunchArgument("use_survey_monitor", default_value="true"),
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.15"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            Node(
                # 【重要】aptの`ros-jazzy-rplidar-ros`の'rplidar_composition'は
                # RPLIDAR S1でSEGVクラッシュするバグを確認した(2026-07-15、
                # Jetson実機)。Slamtec公式`ros2`ブランチをソースビルドした
                # 'rplidar_node'を使うこと(persistent_base.launch.py と同じ)。
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
            ExecuteProcess(
                cmd=[
                    "ros2", "run", "micro_ros_agent", "micro_ros_agent",
                    "serial", "--dev", esp32_port, "-b", "115200",
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
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_laser_tf",
                # 【2026-10-05較正】LiDARの取り付け回転(yaw)。機体4隅の柱(50cm四角)の写り込みから
                # 当てはめて求めた(yaw=-1.75deg=-0.0305rad、当てはめ残差約10mm)。x/yは誤差範囲(約1cm)
                # のため0のまま。引数の順はx y z yaw pitch roll。
                arguments=["0", "0", "0.12", "-0.0305", "0", "0", "base_link", "laser"],
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_tf",
                arguments=[camera_x, camera_y, camera_z, camera_yaw, camera_pitch, camera_roll,
                           "base_link", "camera_link"],
                output="screen",
            ),
            Node(
                package="nokosahen_core",
                executable="person_target_detector",
                name="person_target_detector",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_person_target_detector),
            ),
            # 後ろ向きカメラ2台(左後ろ・右後ろ)の人物検出。person_target_detectorの後ろの方位の確認と、person_clear_gateが使う
            Node(
                package="nokosahen_core",
                executable="rear_camera_persons",
                name="rear_camera_persons",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(PythonExpression(["'", use_person_target_detector, "' == 'true' and '", use_rear_cameras, "' == 'true'"])),
            ),
            Node(
                package="nokosahen_core",
                executable="behavior_director",
                name="behavior_director",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_behavior_director),
            ),
            Node(
                package="nokosahen_core",
                executable="survey_monitor",
                name="survey_monitor",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_survey_monitor),
            ),
        ]
    )
