from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("g1_orchestrator"),
        "config",
        "orchestrator_params.yaml",
    )

    config_arg = DeclareLaunchArgument(
        "orchestrator_config",
        default_value=default_config,
        description="Path to orchestrator parameter YAML file.",
    )

    orchestrator_node = Node(
        package="g1_orchestrator",
        executable="orchestrator_node",
        name="orchestrator_node",
        output="screen",
        parameters=[LaunchConfiguration("orchestrator_config")],
    )

    return LaunchDescription([config_arg, orchestrator_node])
