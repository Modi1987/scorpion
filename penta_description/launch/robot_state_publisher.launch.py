from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import FindExecutable
from launch import LaunchDescription
import os
import subprocess


def generate_launch_description():
    """ Launch args """
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    remapping_arg = DeclareLaunchArgument(
        'joint_states_remappings',
        default_value='/joint_states',
        description='Robot state publisher input topic'
    )
    """ xacro args as launch arguments """
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Prefix for the robot name space'
    )
    enable_gazebo_arg = DeclareLaunchArgument(
        'enable_gazebo',
        default_value='true',
        description='Enable Gazebo simulation, make sure to set use_real_dynamixel_motor to false when set to true'
    )
    use_real_dynamixel_motor_arg = DeclareLaunchArgument(
        'use_real_dynamixel_motor',
        default_value='false',
        description='Enable real Dynamixel motor control, make sure to set enable_gazebo to false when set to true'
    )
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Port name for the DYNAMIXEL hardware interface, change if necessary'
    )

    """ Load URDF """
    urdf_file = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare('penta_description'),
                    'urdf',
                    'penta.urdf.xacro'
                ]
            ),
            ' ',
            'prefix:=',
            LaunchConfiguration('prefix'),
            ' ',
            'use_fake_hardware:=',
            'False',
            ' ',
            'enable_gazebo:=',
            LaunchConfiguration('enable_gazebo'),
            ' ',
            'use_real_dynamixel_motor:=',
            LaunchConfiguration('use_real_dynamixel_motor'),
            ' ',
            'port_name:=',
            LaunchConfiguration('port_name'),
            ' ',
        ]
    )

    """ Lauch description """
    return LaunchDescription([
        name_space_arg,
        remapping_arg,
        prefix_arg,
        enable_gazebo_arg,
        use_real_dynamixel_motor_arg,
        port_name_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('name_space'),
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': urdf_file},
            ],
            remappings=[
                ('joint_states', LaunchConfiguration('joint_states_remappings')),
            ]
        ),
    ])