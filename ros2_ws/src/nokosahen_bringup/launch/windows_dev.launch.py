from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_dir = Path(get_package_share_directory("nokosahen_bringup"))
    params_file = LaunchConfiguration("params_file")
    core_launch = bringup_dir / "launch" / "core.launch.py"

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=str(bringup_dir / "config" / "nokosahen.yaml"),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(core_launch)),
                launch_arguments={
                    "params_file": params_file,
                    "use_mock": "true",
                    "use_camera_counter": "false",
                    "use_person_target_detector": "false",
                    "use_voice_node": "false",
                }.items(),
            ),
        ]
    )

