import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("penta_description"),
        "config",
        "general_config.yaml",
    )
    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="base_twerk",
            executable="base_twerk_action_server_node",
            output="screen",
            parameters=[config],
            remappings=[
                ("cmd_null_setpoint", "/cmd_null_setpoint"),
                ("get_current_null_pose", "/get_current_null_pose"),
            ]
        )
    )
    return ld
