import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare

from nokosahen_core.nav_speed import read_yaml_default

# 【2026-10-19】mapping_mode.launch.py と同じ。Nav2のbt_navigatorが/goal_poseを直接購読して、nav2_goal_router
# (緊急停止・アンケート・範囲の確認、背後の目的地の先回し旋回)を素通りして走り出すのを防ぐため、Nav2側の購読を
# 誰も出さないトピックへ付け替える。
NAV2_UNUSED_GOAL_TOPIC = "/nokosahen/nav2_unused_goal_pose"

# persistent_base.launch.py(ESP32・LiDAR・カメラ・ダッシュボード等)は起動した
# ままで、事前に保存した地図に対してAMCLで位置合わせ(localization)する単位。
# ダッシュボードのMapLifecycleController(dashboard_common.py)が
# `ros2 launch nokosahen_bringup localization_mode.launch.py map:=<保存済みyaml>`
# をサブプロセスとして起動・終了する。
#
# 使い方:
#   ros2 launch nokosahen_bringup localization_mode.launch.py \
#       map:=/path/to/saved_map.yaml


def _truthy(value) -> bool:
    return str(value).strip().lower() in ("true", "1", "yes")


def full_nav_enabled(use_nav2, light) -> bool:
    """Nav2の走行系(+nav2_goal_router・wall_escape・person_clear_gate)を起動するか。
    【2026-11-06】light:=true(軽量の位置推定モード)のときは、use_nav2に関係なく起動しない。
    map_server・amcl(nav2_bringupのlocalization_launch.py)・pose_relay・motion_arbiter は常に起動する。"""
    return _truthy(use_nav2) and not _truthy(light)


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
    use_nav2 = full_nav_enabled(
        LaunchConfiguration("use_nav2").perform(context), LaunchConfiguration("light").perform(context)
    )
    common = {
        "params_file": rewritten,
        "use_sim_time": LaunchConfiguration("use_sim_time").perform(context),
        "autostart": LaunchConfiguration("autostart").perform(context),
    }
    share = get_package_share_directory("nav2_bringup")
    includes = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, "launch", "localization_launch.py")),
            launch_arguments=dict(common, map=LaunchConfiguration("map").perform(context)).items(),
        )
    ]
    if use_nav2:
        includes.append(
            GroupAction(
                [
                    SetRemap(src="goal_pose", dst=NAV2_UNUSED_GOAL_TOPIC),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(os.path.join(share, "launch", "navigation_launch.py")),
                        launch_arguments=common.items(),
                    ),
                ]
            )
        )
    return includes


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    map_yaml = LaunchConfiguration("map")
    use_nav2 = LaunchConfiguration("use_nav2")
    light = LaunchConfiguration("light")
    # use_nav2 が真で、かつ light でないときだけ(通常モード)
    full_nav = PythonExpression(
        ["'", use_nav2, "'.strip().lower() in ('true','1','yes') and '", light, "'.strip().lower() not in ('true','1','yes')"]
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=str(bringup_dir / "config" / "nav2_params_jetson.yaml"),
            ),
            DeclareLaunchArgument(
                "map",
                default_value=str(bringup_dir / "config" / "maps" / "cafeteria.yaml"),
                description="docs/mapping_and_calibration.md の手順で保存した地図",
            ),
            # 【2026-10-07追加】自律走行(Nav2)の最高速度[m/s](mapping_mode.launch.pyと同じ仕組み)。
            DeclareLaunchArgument(
                "nav_max_speed",
                default_value=str(
                    read_yaml_default(str(bringup_dir / "config" / "nokosahen.yaml"))
                ),
            ),
            DeclareLaunchArgument("use_saved_nav_speed", default_value="true"),
            DeclareLaunchArgument("use_nav2", default_value="true"),
            # 【2026-11-06】軽量の位置推定モード(docs/light_localization.md)。true だと Nav2の走行系・nav2_goal_router・
            # wall_escape・person_clear_gate を起動せず、map_server・amcl・pose_relay・motion_arbiter だけにする(CPU削減。走行は使えない)。
            DeclareLaunchArgument("light", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("autostart", default_value="true"),
            OpaqueFunction(function=_nav2_includes),
            # 【2026-10-26】CPU削減: /tf(約70Hz)を直接購読していた自作ノード(nav2_goal_router/wall_escape/motion_arbiter)の代わりに、
            # このノード1つが map→base_link / map→odom を5Hzで /nokosahen/robot_pose へ配る(pose_relay.py)。この3ノードより先に置く。
            Node(
                package="nokosahen_core",
                executable="pose_relay",
                name="pose_relay",
                output="screen",
            ),
            Node(
                package="nokosahen_core",
                executable="nav2_goal_router",
                name="nav2_goal_router",
                output="screen",
                condition=IfCondition(full_nav),
            ),
            # 【2026-10-22】壁際で経路が作れず動かなくなるのを、壁から離れて目的地を出し直して抜ける(wall_escape.py)
            # 【2026-10-22】動いてよいかの判定を1つに集める(docs/motion_priority.md)。wall_escape・代替の回頭は、ここの許可で動く
            Node(
                package="nokosahen_core",
                executable="motion_arbiter",
                name="motion_arbiter",
                output="screen",
            ),
            Node(
                package="nokosahen_core",
                executable="wall_escape",
                name="wall_escape",
                output="screen",
                condition=IfCondition(full_nav),
            ),
            # 【2026-10-22】カメラ(前+後ろ2台)で人がいないと確認できている間だけ、壁際の停止領域(VelocityStop)を無効にする(person_clear_gate.py)
            Node(
                package="nokosahen_core",
                executable="person_clear_gate",
                name="person_clear_gate",
                output="screen",
                condition=IfCondition(full_nav),
            ),
        ]
    )
