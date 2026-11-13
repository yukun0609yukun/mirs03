from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Gazebo本格シミュレーション(gazebo_sim.launch.py)に、実機のjetson_robot.launch.py
# 相当の構成(SLAM/Nav2/nokosahen_core一式/ダッシュボード)を重ねたもの。
# 実機用launchと違い、以下は使わない(Gazebo側が代わりに提供するため):
#   - rplidar_ros(実機LiDARドライバ) -> xacroのray sensorプラグインが/scanを出す
#   - micro_ros_agent / wheel_odom(実機ESP32連携) -> diff_driveプラグインが
#     /odomとodom->base_footprintのTFを直接出す
#   - base_to_laser_tf/base_to_camera_tf(static_transform_publisher) -> xacroの
#     固定ジョイントをrobot_state_publisherがそのまま流すので不要
#
# ARUCOマーカーによる充電ドック復帰は本launchにも含めない(未実装・対象外)。
#
# 使い方:
#   ros2 launch nokosahen_gazebo gazebo_full.launch.py
#   ros2 launch nokosahen_gazebo gazebo_full.launch.py use_camera_counter:=true use_full_dashboard:=true use_stage2_dashboard:=false
#
# 完成版ダッシュボード(残数テーブル・決済横カメラのUI)を確認したい場合:
#   ros2 launch nokosahen_gazebo gazebo_full.launch.py \
#       use_full_dashboard:=true use_stage2_dashboard:=false use_mock_checkout_cams:=true
# use_mock_checkout_cams:=true にすると、実機のRaspberry Pi 4が無くても
# ダミーの決済横カメラ(camera_id: sim-cam-1〜3)がダッシュボードの
# 「決済場所カメラ」欄に映像付きで表示され、メニュー割り当てUIの動作も確認できる
# (nokosahen_checkout/mock_checkout_cam_node.py、シミュレーション専用)。


def generate_launch_description():
    gazebo_dir = Path(get_package_share_directory("nokosahen_gazebo"))
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    gazebo_sim_launch = gazebo_dir / "launch" / "gazebo_sim.launch.py"

    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_slam = LaunchConfiguration("use_slam")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_rviz = LaunchConfiguration("use_rviz")
    world_file = LaunchConfiguration("world_file")
    use_camera_counter = LaunchConfiguration("use_camera_counter")
    use_person_target_detector = LaunchConfiguration("use_person_target_detector")
    use_voice_node = LaunchConfiguration("use_voice_node")
    use_stage2_dashboard = LaunchConfiguration("use_stage2_dashboard")
    use_full_dashboard = LaunchConfiguration("use_full_dashboard")
    front_camera_topic = LaunchConfiguration("front_camera_topic")
    checkout_camera_topic = LaunchConfiguration("checkout_camera_topic")
    use_mock_checkout_cams = LaunchConfiguration("use_mock_checkout_cams")
    mock_checkout_cam_count = LaunchConfiguration("mock_checkout_cam_count")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file", default_value=str(bringup_dir / "config" / "nokosahen.yaml")
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=str(bringup_dir / "config" / "nav2_params_jetson.yaml"),
            ),
            DeclareLaunchArgument(
                "slam_params_file",
                default_value=str(bringup_dir / "config" / "slam_toolbox_online_async.yaml"),
            ),
            DeclareLaunchArgument("use_slam", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("autostart", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "world_file", default_value=str(gazebo_dir / "worlds" / "cafeteria.world")
            ),
            # Stage3(残数カウント)も含めてGazebo上で試したい場合はtrueにする。
            DeclareLaunchArgument("use_camera_counter", default_value="false"),
            DeclareLaunchArgument("use_person_target_detector", default_value="true"),
            # VOICEVOX/Ollamaはホスト常駐サービス前提(docs/setup_docker.md)なので、
            # シミュレーションPC側にvoice_bridgeが無ければfalseにしておく。
            DeclareLaunchArgument("use_voice_node", default_value="false"),
            DeclareLaunchArgument("use_stage2_dashboard", default_value="true"),
            DeclareLaunchArgument("use_full_dashboard", default_value="false"),
            DeclareLaunchArgument(
                "front_camera_topic",
                default_value="/nokosahen/sim/front_camera/image_raw",
            ),
            DeclareLaunchArgument(
                "checkout_camera_topic",
                default_value="/nokosahen/sim/checkout_camera/image_raw",
            ),
            # 実機のRaspberry Pi 4(nokosahen_checkout)無しで、完成版ダッシュボードの
            # 決済横カメラUI(映像プレビュー・メニュー割り当て)を確認するためのダミー。
            # 本番では使わない(mock_checkout_cam_node.py参照)。
            DeclareLaunchArgument("use_mock_checkout_cams", default_value="false"),
            DeclareLaunchArgument("mock_checkout_cam_count", default_value="3"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(gazebo_sim_launch)),
                launch_arguments={
                    "world_file": world_file,
                    "use_rviz": use_rviz,
                    "use_sim_time": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
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
                        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
                    )
                ),
                launch_arguments={
                    "params_file": nav2_params_file,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                }.items(),
                condition=IfCondition(use_nav2),
            ),
            Node(
                package="nokosahen_core",
                executable="nav2_goal_router",
                name="nav2_goal_router",
                output="screen",
                condition=IfCondition(use_nav2),
            ),
            Node(
                package="nokosahen_core",
                executable="inventory_manager",
                name="inventory_manager",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nokosahen_core",
                executable="web_ui",
                name="web_ui",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nokosahen_core",
                executable="crowd_monitor",
                name="crowd_monitor",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nokosahen_core",
                executable="behavior_director",
                name="behavior_director",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nokosahen_core",
                executable="person_target_detector",
                name="person_target_detector",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "camera_image_topic": front_camera_topic,
                    },
                ],
                condition=IfCondition(use_person_target_detector),
            ),
            Node(
                package="nokosahen_core",
                executable="camera_counter",
                name="camera_counter",
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "camera_image_topic": checkout_camera_topic,
                    },
                ],
                condition=IfCondition(use_camera_counter),
            ),
            Node(
                package="nokosahen_voice",
                executable="voice_node",
                name="voice_node",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                condition=IfCondition(use_voice_node),
            ),
            Node(
                package="nokosahen_core",
                executable="stage2_dashboard",
                name="stage2_dashboard",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                condition=IfCondition(use_stage2_dashboard),
            ),
            Node(
                package="nokosahen_core",
                executable="full_dashboard",
                name="full_dashboard",
                output="screen",
                parameters=[params_file, {"use_sim_time": use_sim_time}],
                condition=IfCondition(use_full_dashboard),
            ),
            Node(
                package="nokosahen_checkout",
                executable="mock_checkout_cam",
                name="mock_checkout_cam",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "camera_count": mock_checkout_cam_count,
                    }
                ],
                condition=IfCondition(use_mock_checkout_cams),
            ),
        ]
    )
