from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("g1_locomotion"),
        "config",
        "locomotion_params.yaml",
    )

    config_arg = DeclareLaunchArgument(
        "locomotion_config",
        default_value=default_config,
        description="Path to locomotion parameter YAML file.",
    )
    embedded_sim_policy_arg = DeclareLaunchArgument(
        "embedded_sim_policy",
        default_value="false",
        description="Publish embedded simulator walk commands instead of rt/lowcmd.",
    )

    locomotion_node = Node(
        package="g1_locomotion",
        executable="locomotion_node",
        name="locomotion_node",
        output="screen",
        parameters=[
            LaunchConfiguration("locomotion_config"),
            {"embedded_sim_policy": LaunchConfiguration("embedded_sim_policy")},
        ],
    )

    return LaunchDescription([config_arg, embedded_sim_policy_arg, locomotion_node])
