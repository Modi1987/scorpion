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
            package="gait_generator",
            executable="gait_generator_node",
            output="screen",
            parameters=[config],
            namespace=LaunchConfiguration("name_space"),
            remappings=[
                # consumed topics
                ("cmd_vel", "cmd_vel"),
                ("null_space_pose", "null_space_pose"),
            ],
        ),
    )
    ld.add_action(
        Node(
            package="gait_generator",
            executable="base_tf_broadcaster_node",
            output="screen",
            parameters=[config],
            namespace=LaunchConfiguration("name_space"),
            remappings=[
                # output topics
                ("null_space_pose", "null_space_pose"),
            ],
        )
    )
    return ld
