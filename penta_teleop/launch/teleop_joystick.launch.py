from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    joy_params = os.path.join(
        get_package_share_directory("penta_teleop"), "config", "teleop_joystick.yaml"
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[joy_params],
    )

    teleop_twist_joy_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_node",
        parameters=[joy_params],
        # remappings=[("/cmd_vel", "/input/cmd_vel_teleop_joy")],
    )

    joystick_extra_controls_node = Node(
        package="penta_teleop",
        executable="joystick_extra_controls_node",
        name="joystick_extra_controls_node",
        parameters=[joy_params],
    )

    return LaunchDescription(
        [
            joy_node,
            teleop_twist_joy_node,
            joystick_extra_controls_node
        ]
    )
