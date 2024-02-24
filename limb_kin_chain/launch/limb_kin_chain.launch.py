import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("differential_drive"),
        "config",
        "topperware_bot.yaml",
    )
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="differential_drive",
                executable="differential_drive_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
