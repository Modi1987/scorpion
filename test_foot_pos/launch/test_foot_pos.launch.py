import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("penta_pod"),
        "config",
        "limb_0.yaml",
    )
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                namespace='limb0', # to animate limb0
                package="test_foot_pos",
                executable="test_foot_pos_node",
                output="screen",
                parameters=[config],
            ),
            launch_ros.actions.Node(
                namespace='limb1', # to animate limb1
                package="test_foot_pos",
                executable="test_foot_pos_node",
                output="screen",
                parameters=[config],
            ),
            launch_ros.actions.Node(
                namespace='limb2', # to animate limb2
                package="test_foot_pos",
                executable="test_foot_pos_node",
                output="screen",
                parameters=[config],
            ),
            launch_ros.actions.Node(
                namespace='limb3', # to animate limb3
                package="test_foot_pos",
                executable="test_foot_pos_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
