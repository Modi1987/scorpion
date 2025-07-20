import launch
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    ld = launch.LaunchDescription()

    """ Launch args """
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    ld.add_action(name_space_arg)

    config_file = os.path.join(
        get_package_share_directory("penta_description"),
        "config",
        "general_config.yaml",
    )
    
    setpoint_to_frward_joint_position = Node(
        package="ros_to_ros2_control_command_bridge",
        executable="forward_joint_command_bridge_node",
        output="both",
        remappings={
            (
                "actuator_setpoints",
                "/forward_position_controller/commands",
            ),  # this is for the output topic
            ("joint_setpoints", "/joint_setpoints"),
        },  # this is for the input topic
        parameters=[
            {"name_space": launch.substitutions.LaunchConfiguration("name_space")},
            config_file,
        ],
    )
    ld.add_action(setpoint_to_frward_joint_position)

    return ld
