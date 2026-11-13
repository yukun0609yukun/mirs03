import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from nokosahen_core.nav_speed import read_yaml_default

# 【2026-10-19】Nav2のbt_navigatorは、標準で/goal_poseを自分でも購読し、受け取るとすぐ自分でNavigateToPoseを始める
# (RViz用)。このプロジェクトの/goal_poseはnav2_goal_routerが受けて、緊急停止・アンケート・走行可能範囲の確認、
# 背後の目的地の先回し旋回をしたうえでNav2へ送る。bt_navigatorが同じ/goal_poseを直接受けると、ルーターを通らずに
# 目的地へ走り出し(旋回と競合・確認の素通し・同じ目的地の二重送信で互いにpreempt)、「後ろの目的地へ行けない」
# 「走り出すまで遅い」の原因になる(実機ログ: ルーターの旋回判断の7ms後にbt_navigatorが直接走り出していた)。
# Nav2側の購読を、誰も出さないトピックへ付け替えて、目的地の入口をルーターだけにする。
NAV2_UNUSED_GOAL_TOPIC = "/nokosahen/nav2_unused_goal_pose"

# persistent_base.launch.py(ESP32・LiDAR・カメラ・ダッシュボード等)は起動した
# ままで、「地図作成(SLAM)」だけを後から開始・停止できるようにする単位。
# ダッシュボードのMapLifecycleController(dashboard_common.py)が
# `ros2 launch nokosahen_bringup mapping_mode.launch.py` をサブプロセスとして
# 起動・終了する(このプロセスを止めても、persistent_base側は動き続ける)。
#
# 使い方:
#   ros2 launch nokosahen_bringup mapping_mode.launch.py


def _nav2_includes(context, *args, **kwargs):
    """nav2_bringupを、Nav2最高速度(nav_max_speed)を反映した一時のparamsコピーで起動する。

    速度に関わる値(DWB・velocity_smoother・collision_monitor・ローカルコストマップ)は
    nokosahen_core.nav_speedの1つの変換でまとめて書き換える(ダッシュボードの実行時変更と
    同じ関数)。優先順位: ダッシュボードが記憶した値(12時間以内・use_saved_nav_speed:=true)
    > nav_max_speed引数 > 0.5。"""
    from nokosahen_core.nav_speed import resolve_launch_nav_speed, write_rewritten_params

    src = LaunchConfiguration("nav2_params_file").perform(context)
    speed, source = resolve_launch_nav_speed(
        LaunchConfiguration("nav_max_speed").perform(context),
        LaunchConfiguration("use_saved_nav_speed").perform(context).lower() in ("true", "1", "yes"),
    )
    rewritten = write_rewritten_params(src, speed)
    print(f"[nav_speed] Nav2最高速度 {speed:g} m/s (出所: {source}) -> {rewritten}")
    use_nav2 = LaunchConfiguration("use_nav2").perform(context).lower() in ("true", "1", "yes")
    common = {
        "params_file": rewritten,
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
        "autostart": LaunchConfiguration("autostart").perform(context),
    }
    includes = []
    if use_nav2:
        includes.append(
            GroupAction(
                [
                    SetRemap(src="goal_pose", dst=NAV2_UNUSED_GOAL_TOPIC),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(get_package_share_directory("nav2_bringup"), "launch", "navigation_launch.py")
                        ),
                        launch_arguments=common.items(),
                    ),
                ]
            )
        )
    return includes


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    params_file = LaunchConfiguration("params_file")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    slam_params_file = LaunchConfiguration("slam_params_file")
    use_nav2 = LaunchConfiguration("use_nav2")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    use_frontier_explorer = LaunchConfiguration("use_frontier_explorer")

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
            # 【2026-10-07追加】自律走行(Nav2)の最高速度[m/s]。1つの値からDWB・velocity_smoother・
            # collision_monitor・ローカルコストマップをまとめて決める(docs/autonomous_speed.md)。
            # 既定はnokosahen.yamlのnokosahen_launch.nav_max_speed(0.5)。上限1.1・下限0.1。
            DeclareLaunchArgument(
                "nav_max_speed",
                default_value=str(
                    read_yaml_default(str(bringup_dir / "config" / "nokosahen.yaml"))
                ),
            ),
            # trueなら、ダッシュボードが「Nav2未起動のとき」に記憶した速度(~/.nokosahen_nav_speed.json、
            # 12時間以内)を引数より優先する。引数で固定したいときはfalse。
            DeclareLaunchArgument("use_saved_nav_speed", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            # 自律探索マッピング(frontier_explorer)自体は既定で無効
            # (/nokosahen/frontier_explorer/enabledで有効化するまで待機する)。
            # ノードは常に起動しておき、ダッシュボードのボタンで
            # 有効/無効を切り替えられるようにする。
            DeclareLaunchArgument("use_frontier_explorer", default_value="true"),
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
            ),
            OpaqueFunction(function=_nav2_includes),
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
            # 【2026-10-22】壁際で経路が作れず動かなくなるのを、壁から離れて目的地を出し直して抜ける(wall_escape.py)
            # 【2026-10-22】動いてよいかの判定を1つに集める(docs/motion_priority.md)。wall_escape・代替の回頭は、ここの許可で動く
            Node(
                package="nokosahen_core",
                executable="motion_arbiter",
                name="motion_arbiter",
                output="screen",
                condition=IfCondition(use_nav2),
            ),
            Node(
                package="nokosahen_core",
                executable="wall_escape",
                name="wall_escape",
                output="screen",
                condition=IfCondition(use_nav2),
            ),
            # 【2026-10-22】カメラ(前+後ろ2台)で人がいないと確認できている間だけ、壁際の停止領域(VelocityStop)を無効にする(person_clear_gate.py)
            Node(
                package="nokosahen_core",
                executable="person_clear_gate",
                name="person_clear_gate",
                output="screen",
                condition=IfCondition(use_nav2),
            ),
            Node(
                package="nokosahen_core",
                executable="frontier_explorer",
                name="frontier_explorer",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_frontier_explorer),
            ),
        ]
    )
