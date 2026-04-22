from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("g1_bridge"),
        "config",
        "bridge_config.yaml",
    )

    config_arg = DeclareLaunchArgument(
        "bridge_config",
        default_value=default_config,
        description="Path to bridge parameter YAML file.",
    )

    bridge_node = Node(
        package="g1_bridge",
        executable="sdk_bridge_node",
        name="sdk_bridge",
        output="screen",
        parameters=[LaunchConfiguration("bridge_config")],
    )

    return LaunchDescription([config_arg, bridge_node])
