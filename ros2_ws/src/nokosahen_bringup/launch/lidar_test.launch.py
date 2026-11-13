from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# LiDAR単体動作試験用の最小構成launch。
# ESP32・SLAM/Nav2・カメラ・ダッシュボード類は一切起動しない。
# rplidar_ros(ドライバ)+ lidar_test_dashboard(ブラウザ確認用の最小UI)のみ。
# 「本体を組み上げる前に、LiDAR単体がまず正しく動いているか」を素早く
# 確認するためのもの(docs/mapping_and_calibration.md参照)。


def generate_launch_description():
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_baudrate = LaunchConfiguration("lidar_baudrate")
    use_dashboard = LaunchConfiguration("use_dashboard")
    dashboard_port = LaunchConfiguration("dashboard_port")
    camera_index = LaunchConfiguration("camera_index")
    camera_fps = LaunchConfiguration("camera_fps")

    return LaunchDescription(
        [
            # RPLIDAR S1(360度)固有の値。A1/A2/A3とはボーレート・スキャン仕様が異なる。
            DeclareLaunchArgument("lidar_port", default_value="/dev/rplidar"),
            DeclareLaunchArgument("lidar_baudrate", default_value="256000"),
            DeclareLaunchArgument("use_dashboard", default_value="true"),
            DeclareLaunchArgument("dashboard_port", default_value="8099"),
            # Webカメラ(/dev/webcam_checkout等の固定名、または0/1等のindex)。
            # ROS2トピック経由ではなくOpenCVで直接開くだけの単体確認用。
            DeclareLaunchArgument("camera_index", default_value="0"),
            DeclareLaunchArgument("camera_fps", default_value="5.0"),
            Node(
                # 【重要】aptの`ros-jazzy-rplidar-ros`(2.1.0)が提供する
                # 'rplidar_composition'実行ファイルは、RPLIDAR S1で
                # startMotor()呼び出し時にSEGVでクラッシュするバグを
                # 2026-07-15にJetson実機で確認した(legacyな
                # rplidar_driver.cppのドライバパスが原因と見られる)。
                # 対策としてSlamtec公式リポジトリの`ros2`ブランチを
                # ソースビルドした`rplidar_node`(新しいsl_lidar_driver
                # SDK経由)を使う。scripts/install_rplidar_from_source.sh
                # 参照。このソースビルド分は別workspace
                # (~/lidar_src_build)としてオーバーレイし、このlaunch実行前に
                # `source ~/lidar_src_build/install/setup.bash`しておくこと。
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
            ),
            Node(
                package="nokosahen_core",
                executable="scan_self_filter",
                name="scan_self_filter",
                output="screen",
            ),
            Node(
                package="nokosahen_core",
                executable="lidar_test_dashboard",
                name="lidar_test_dashboard",
                output="screen",
                parameters=[{
                    "port": dashboard_port,
                    "camera_index": camera_index,
                    "camera_fps": camera_fps,
                }],
                condition=IfCondition(use_dashboard),
            ),
        ]
    )
