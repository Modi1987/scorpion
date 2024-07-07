import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("penta_description"),
        "config",
        "legs_body_transforms.yaml",
    )
    ld = LaunchDescription()
    ld.add_action(Node(
        package="gait_generator",
        executable="gait_generator_node",
        output="screen",
        parameters=[config],
    ))
    return ld
