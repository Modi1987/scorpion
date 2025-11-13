import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch argument 'mode'
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Choose operation mode: real or virtual'
    )

    # Path to the config file
    config = os.path.join(
        get_package_share_directory('penta_description'),
        'config',
        'general_config.yaml'
    )

    # Setpoint topic arg
    joints_setpoint_topic_arg = DeclareLaunchArgument(
        'joints_setpoint_topic',
        default_value='joint_states',
        description='Topic for joint setpoints'
    )
    joints_setpoint_topic = LaunchConfiguration('joints_setpoint_topic')

    # Node definition for Penta HiWonder actuators
    penta_hiwonder_actuators_node = Node(
        package='penta_hiwonder_actuators',
        executable='penta_hiwonder_actuators_node',
        output='screen',
        parameters=[config],
        # Pass 'mode' argument to the node
        arguments=[LaunchConfiguration('mode')],
        remappings=[
            # published
            ('actuator_setpoint_degree', 'actuator_setpoint_degree'),
            ('actuator_ticks', 'actuator_ticks'),
            # subscribed
            ('joint_states', joints_setpoint_topic)
        ]
    )

    # Create a launch description and add the actions
    ld = LaunchDescription([
        mode_arg,            # Add the mode argument
        joints_setpoint_topic_arg,
        penta_hiwonder_actuators_node   # Add the node
    ])

    return ld
