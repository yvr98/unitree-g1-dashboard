from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    bridge_launch = os.path.join(
        get_package_share_directory("g1_bridge"),
        "launch",
        "bridge.launch.py",
    )
    safety_launch = os.path.join(
        get_package_share_directory("g1_safety"),
        "launch",
        "safety.launch.py",
    )
    orchestrator_launch = os.path.join(
        get_package_share_directory("g1_orchestrator"),
        "launch",
        "orchestrator.launch.py",
    )
    locomotion_launch = os.path.join(
        get_package_share_directory("g1_locomotion"),
        "launch",
        "locomotion.launch.py",
    )
    api_launch = os.path.join(
        get_package_share_directory("g1_api"),
        "launch",
        "api.launch.py",
    )

    embedded_sim_policy_arg = DeclareLaunchArgument(
        "embedded_sim_policy",
        default_value="false",
        description="Route locomotion commands to the simulator embedded controller.",
    )

    return LaunchDescription(
        [
            embedded_sim_policy_arg,
            IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(safety_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(orchestrator_launch)
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(locomotion_launch),
                launch_arguments={
                    "embedded_sim_policy": LaunchConfiguration("embedded_sim_policy")
                }.items(),
            ),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(api_launch)),
        ]
    )
