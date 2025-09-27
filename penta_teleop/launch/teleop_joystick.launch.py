from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory
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

    joy_params = os.path.join(
        get_package_share_directory("penta_teleop"), "config", "teleop_joystick.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        namespace=LaunchConfiguration("name_space"),
        parameters=[joy_params],
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        namespace=LaunchConfiguration("name_space"),
        parameters=[joy_params],
        remappings=[("cmd_vel", "cmd_vel_not_smoothed")],
    )

    joystick_extra_controls_configs = os.path.join(
        get_package_share_directory("penta_teleop"), "config", "joy_extra_controls.yaml"
    )

    general_config_params = os.path.join(
        get_package_share_directory("penta_description"), "config", "general_config.yaml"
    )

    joystick_extra_controls_node = Node(
        package="penta_teleop",
        executable="joystick_extra_controls_node",
        name="joystick_extra_controls_node",
        namespace=LaunchConfiguration("name_space"),
        remappings=[
            # services
            ("cmd_null_setpoint", "base_twerk/cmd_null_setpoint"),
            ("get_current_null_pose", "base_twerk/get_current_null_pose"),
        ],
        parameters=[joystick_extra_controls_configs, general_config_params],
    )

    cmd_vel_smoothing_configs = os.path.join(
        get_package_share_directory("penta_teleop"), "config", "cmd_vel_smoothing.yaml"
    )
    cmd_vel_smoothing_node = Node(
        package="penta_teleop",
        executable="cmd_vel_smoothing_node",
        name="cmd_vel_smoothing_node",
        namespace=LaunchConfiguration("name_space"),
        remappings=[
            ("cmd_vel", "cmd_vel_not_smoothed"),
            ("smoothed_cmd_vel", "cmd_vel"),
        ],
        parameters=[cmd_vel_smoothing_configs],
    )

    nodes = [
        joy_node,
        teleop_twist_joy_node,
        joystick_extra_controls_node,
        cmd_vel_smoothing_node,
    ]
    for node in nodes:
        ld.add_action(node)

    return ld
