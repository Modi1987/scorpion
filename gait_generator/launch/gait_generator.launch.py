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
            package="gait_generator",
            executable="gait_generator_node",
            output="screen",
            parameters=[config],
            remappings=[
                # consumed topics
                ("cmd_vel", "/cmd_vel"),
                ("null_space_pose", "/null_space_pose"),
            ],
        ),
    )
    ld.add_action(
        Node(
            package="gait_generator",
            executable="base_tf_broadcaster_node",
            output="screen",
            parameters=[config],
            remappings=[
                # output topics
                ("null_space_pose", "/null_space_pose"),
            ],
        )
    )
    return ld
