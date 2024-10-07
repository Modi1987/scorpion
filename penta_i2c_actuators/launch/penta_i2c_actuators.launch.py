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
        default_value='virtual',
        description='Choose operation mode: real or virtual'
    )

    # Path to the config file
    config = os.path.join(
        get_package_share_directory('penta_description'),
        'config',
        'general_config.yaml'
    )

    # Node definition for I2C actuators
    i2c_actuators_node = Node(
        package='penta_i2c_actuators',
        executable='penta_i2c_actuators_node',
        output='screen',
        parameters=[config],
        # Pass 'mode' argument to the node
        arguments=[LaunchConfiguration('mode')]
    )

    # Create a launch description and add the actions
    ld = LaunchDescription([
        mode_arg,            # Add the mode argument
        i2c_actuators_node   # Add the node
    ])

    return ld
