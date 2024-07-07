import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("penta_description"),
        "config",
        "limb_0_mdh.yaml",
    )
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                package="limb_kin_chain",
                executable="limb_kin_chain_node",
                output="screen",
                parameters=[config],
            ),
        ]
    )
