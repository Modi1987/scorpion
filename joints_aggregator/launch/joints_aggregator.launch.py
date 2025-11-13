import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    ld = LaunchDescription()

    """ Launch args """
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    ld.add_action(name_space_arg)

    """ Nodes """
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
                namespace=LaunchConfiguration("name_space"),
                parameters=[config],
                remappings=[
                    ("joint_setpoints", "joint_setpoints"),
                ],
            ),
        ]
    )
