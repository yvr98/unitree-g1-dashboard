from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("g1_safety"),
        "config",
        "safety_params.yaml",
    )

    config_arg = DeclareLaunchArgument(
        "safety_config",
        default_value=default_config,
        description="Path to safety monitor parameter YAML file.",
    )

    safety_node = Node(
        package="g1_safety",
        executable="safety_monitor_node",
        name="safety_monitor_node",
        output="screen",
        parameters=[LaunchConfiguration("safety_config")],
    )

    return LaunchDescription([config_arg, safety_node])
