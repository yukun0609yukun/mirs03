from setuptools import find_packages, setup

package_name = "nokosahen_core"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MIRS",
    maintainer_email="mirs@example.local",
    description="Core ROS 2 nodes for the Nokosahen cafeteria guide robot PoC.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "behavior_director = nokosahen_core.behavior_director:main",
            "camera_counter = nokosahen_core.camera_counter:main",
            "crowd_monitor = nokosahen_core.crowd_monitor:main",
            "frontier_explorer = nokosahen_core.frontier_explorer:main",
            "inventory_manager = nokosahen_core.inventory_manager:main",
            "mock_sensors = nokosahen_core.mock_sensors:main",
            "nav2_goal_router = nokosahen_core.nav2_goal_router:main",
            "pose_relay = nokosahen_core.pose_relay:main",
            "wall_escape = nokosahen_core.wall_escape:main",
            "motion_arbiter = nokosahen_core.motion_arbiter:main",
            "rear_camera_persons = nokosahen_core.rear_camera_persons:main",
            "person_clear_gate = nokosahen_core.person_clear_gate:main",
            "person_target_detector = nokosahen_core.person_target_detector:main",
            "web_ui = nokosahen_core.web_ui_node:main",
            "wheel_odom = nokosahen_core.wheel_odom_node:main",
            "stage1_dashboard = nokosahen_core.stage1_dashboard_node:main",
            "stage2_dashboard = nokosahen_core.stage2_dashboard_node:main",
            "full_dashboard = nokosahen_core.full_dashboard_node:main",
            "lidar_test_dashboard = nokosahen_core.lidar_test_dashboard:main",
            "menu_schedule = nokosahen_core.menu_schedule_node:main",
            "scan_self_filter = nokosahen_core.scan_self_filter:main",
            "survey_monitor = nokosahen_core.survey_monitor:main",
            "announce_node = nokosahen_core.announce_node:main",
            "device_watchdog = nokosahen_core.device_watchdog:main",
        ],
    },
)
