from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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

    return LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource(bridge_launch)),
            IncludeLaunchDescription(PythonLaunchDescriptionSource(safety_launch)),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(orchestrator_launch)
            ),
        ]
    )
