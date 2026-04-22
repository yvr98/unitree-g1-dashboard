from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description() -> LaunchDescription:
    default_config = os.path.join(
        get_package_share_directory("g1_api"),
        "config",
        "api_config.yaml",
    )

    config_arg = DeclareLaunchArgument(
        "api_config",
        default_value=default_config,
        description="Path to API parameter YAML file.",
    )

    rosbridge_port_arg = DeclareLaunchArgument(
        "rosbridge_port",
        default_value="9090",
        description="Port for rosbridge websocket server.",
    )

    rosbridge_node = Node(
        package="rosbridge_server",
        executable="rosbridge_websocket",
        name="rosbridge_websocket",
        output="screen",
        parameters=[{"port": LaunchConfiguration("rosbridge_port")}],
    )

    api_node = Node(
        package="g1_api",
        executable="api_server",
        name="g1_api_node",
        output="screen",
        parameters=[LaunchConfiguration("api_config")],
    )

    shutdown_on_rosbridge_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=rosbridge_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="rosbridge_websocket exited; shutting down API launch."
                    )
                )
            ],
        )
    )
    shutdown_on_api_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=api_node,
            on_exit=[
                EmitEvent(
                    event=Shutdown(
                        reason="api_server exited; shutting down API launch."
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            config_arg,
            rosbridge_port_arg,
            rosbridge_node,
            api_node,
            shutdown_on_rosbridge_exit,
            shutdown_on_api_exit,
        ]
    )
