import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("penta_pod"),
        "config",
        "limb_0.yaml",
    )
    ld = LaunchDescription()
    for i in range(5):
        limb_prefix = f'limb{i}'
        ld.add_action(Node(
            namespace=limb_prefix,
            package="test_foot_pos",
            executable="test_foot_pos_node",
            output="screen",
            parameters=[config],
        ))
    return ld
