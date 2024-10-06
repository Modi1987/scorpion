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

    the_node = Node(
                    package='penta_i2c_actuators',
                    executable='penta_i2c_actuators_node',
                    output='screen',
                    parameters=[config],
                )
    ld = LaunchDescription()
    ld.add_action(the_node)
    return ld
