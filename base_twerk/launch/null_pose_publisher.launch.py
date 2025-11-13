import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
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
    ld.add_action(
        Node(
            package="base_twerk",
            executable="base_twerk_publisher_node",
            output="screen",
            namespace=LaunchConfiguration("name_space"),
            parameters=[config],
            remappings=[
                # services
                ("cmd_null_setpoint", "base_twerk/cmd_null_setpoint"),
                ("get_current_null_pose", "base_twerk/get_current_null_pose"),
                # topics
                ("null_space_pose", "null_space_pose")
            ],
        )
    )
    return ld
