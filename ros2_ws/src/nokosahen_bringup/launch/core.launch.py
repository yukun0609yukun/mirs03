from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def core_nodes(params_file):
    return [
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
    ]


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    default_params = str(bringup_dir / "config" / "nokosahen.yaml")

    params_file = LaunchConfiguration("params_file")
    use_mock = LaunchConfiguration("use_mock")
    use_behavior_director = LaunchConfiguration("use_behavior_director")
    use_camera_counter = LaunchConfiguration("use_camera_counter")
    use_person_target_detector = LaunchConfiguration("use_person_target_detector")
    use_rear_cameras = LaunchConfiguration("use_rear_cameras")
    use_voice_node = LaunchConfiguration("use_voice_node")
    use_stage2_dashboard = LaunchConfiguration("use_stage2_dashboard")
    use_full_dashboard = LaunchConfiguration("use_full_dashboard")
    use_survey_monitor = LaunchConfiguration("use_survey_monitor")
    # docs/mapping_and_calibration.md の較正結果をここへ反映する
    # (base_link から見たカメラの取り付け位置・角度)。
    camera_x = LaunchConfiguration("camera_x")
    camera_y = LaunchConfiguration("camera_y")
    camera_z = LaunchConfiguration("camera_z")
    camera_roll = LaunchConfiguration("camera_roll")
    camera_pitch = LaunchConfiguration("camera_pitch")
    camera_yaw = LaunchConfiguration("camera_yaw")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("use_mock", default_value="false"),
            # 【2026-07-22追加】ラジコン運用のみの階層(自律行動なし)と、
            # 自律行動あり(見回し・接近・案内)の階層を launch 引数で切り替え
            # られるようにする(ユーザー要望: 両方の起動方法を分けておきたい)。
            # scripts/start_persistent_base_rc_only.sh はこれをfalseにして呼ぶ。
            DeclareLaunchArgument("use_behavior_director", default_value="true"),
            DeclareLaunchArgument("use_camera_counter", default_value="false"),
            DeclareLaunchArgument("use_person_target_detector", default_value="true"),
            # 【2026-10-22】後ろ向きカメラ(左後ろ・右後ろ)の人物検出。カメラが無い/外した構成では false(既定 true)。
            # false/無しの構成は従来どおり(前面カメラ+LiDAR。壁際の停止領域は人の有無に関わらず常に有効)。
            DeclareLaunchArgument("use_rear_cameras", default_value="true"),
            DeclareLaunchArgument("use_voice_node", default_value="true"),
            # Stage2用(残数カウント無効)とcompleteな完成版(残数カウント有効)は
            # それぞれ別のダッシュボードを使うため、同時にtrueにはしない
            # (どちらもポート8090を使う想定なので、上位launchファイル側で
            #  片方だけ有効化する)。
            DeclareLaunchArgument("use_stage2_dashboard", default_value="false"),
            DeclareLaunchArgument("use_full_dashboard", default_value="false"),
            # 【2026-10-09追加】ラズパイのアンケート回答中にロボットを止める橋渡し(docs/survey_robot_hold.md)。
            DeclareLaunchArgument("use_survey_monitor", default_value="true"),
            DeclareLaunchArgument("camera_x", default_value="0.0"),
            DeclareLaunchArgument("camera_y", default_value="0.0"),
            DeclareLaunchArgument("camera_z", default_value="0.15"),
            DeclareLaunchArgument("camera_roll", default_value="0.0"),
            DeclareLaunchArgument("camera_pitch", default_value="0.0"),
            DeclareLaunchArgument("camera_yaw", default_value="0.0"),
            *core_nodes(params_file),
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
                executable="mock_sensors",
                name="mock_sensors",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_mock),
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
                package="nokosahen_core",
                executable="person_target_detector",
                name="person_target_detector",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(use_person_target_detector),
            ),
            # 【2026-10-22】後ろ向きカメラ2台(左後ろ・右後ろ)の人物検出。person_target_detectorの後ろの方位の確認と、person_clear_gateが使う
            Node(
                package="nokosahen_core",
                executable="rear_camera_persons",
                name="rear_camera_persons",
                output="screen",
                parameters=[params_file],
                condition=IfCondition(PythonExpression(["'", use_person_target_detector, "' == 'true' and '", use_rear_cameras, "' == 'true'"])),
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
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_camera_tf",
                arguments=[camera_x, camera_y, camera_z, camera_yaw, camera_pitch, camera_roll,
                           "base_link", "camera_link"],
                output="screen",
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

