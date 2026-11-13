from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Gazebo classic(gazebo_ros)によるロボット単体シミュレーション。
# 差動駆動(libgazebo_ros_diff_drive)・RPLIDAR S1相当のray sensor・
# 正面カメラ(front_camera)をxacroに定義し、決済横カメラ
# (checkout_camera、ロボット非搭載の固定カメラ)はworlds/cafeteria.world側に
# 定義している(実機の構成と同じ切り分け)。
#
# これ単体ではNav2/SLAM/nokosahen_coreは起動しない(ロボットとLiDAR/カメラの
# 動作確認・RViz表示確認用)。Nav2やnokosahen_coreも含めて動かす場合は
# gazebo_full.launch.pyを使う。
#
# 使い方:
#   ros2 launch nokosahen_gazebo gazebo_sim.launch.py
#   ros2 launch nokosahen_gazebo gazebo_sim.launch.py use_rviz:=true


def generate_launch_description():
    gazebo_pkg_dir = Path(get_package_share_directory("nokosahen_gazebo"))
    xacro_file = gazebo_pkg_dir / "urdf" / "nokosahen.urdf.xacro"
    default_world = gazebo_pkg_dir / "worlds" / "cafeteria.world"
    default_rviz_config = gazebo_pkg_dir / "rviz" / "nokosahen.rviz"

    world_file = LaunchConfiguration("world_file")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = Command(["xacro ", str(xacro_file)])

    return LaunchDescription(
        [
            DeclareLaunchArgument("world_file", default_value=str(default_world)),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"]
                    )
                ),
                launch_arguments={"world": world_file}.items(),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{
                    "robot_description": robot_description,
                    "use_sim_time": use_sim_time,
                }],
            ),
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                name="spawn_nokosahen",
                output="screen",
                arguments=[
                    "-topic", "robot_description",
                    "-entity", "nokosahen",
                    # WSL2/WSLg環境ではgzserverの起動(/spawn_entityサービスの
                    # advertiseまで)が既定の30秒を超えることがあり、
                    # "Service /spawn_entity unavailable"で失敗する。
                    # 余裕を持ったタイムアウトにしておく。
                    "-timeout", "90",
                    # (0,-2)は決済カウンター(約1.9m)や南東側の壁(約1.78m)が
                    # crowd_monitorのnear_distance_m(2.0m)圏内に入ってしまい、
                    # 起動直後からcrowded判定に固定される配置だったため変更。
                    # ここは壁・カウンター・人物モデルいずれからも2m以上離れている。
                    "-x", "-1.0", "-y", "-0.5", "-z", "0.05",
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", str(default_rviz_config)],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
