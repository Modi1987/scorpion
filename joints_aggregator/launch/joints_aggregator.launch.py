import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("penta_description"),
        "config",
        "general_config.yaml",
    )
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="joints_aggregator",
                executable="joints_aggregator_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
