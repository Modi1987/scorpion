import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Namespace for the node"
    )

    return LaunchDescription(
        [
            name_space_arg,
            Node(
                package="twist_to_odom",
                executable="twist_to_odom_node",
                namespace=LaunchConfiguration("name_space"),
                output="screen",
                remappings=[
                    ("feedback_cmd_vel", "feedback_cmd_vel"),
                    ("odom", "odom"),
                ],
            ),
        ]
    )
