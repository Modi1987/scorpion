import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

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
    # Declare the launch argument 'mode'
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='virtual',
        description='Choose operation mode: real or virtual'
    )
    ld.add_action(mode_arg)

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
        namespace=LaunchConfiguration('name_space'),
        # Pass 'mode' argument to the node
        arguments=[LaunchConfiguration('mode')],
        remappings=[
            ('joint_setpoints', 'joint_setpoints'),
            ('actuator_setpoint_degree', 'actuator_setpoint_degree'),
        ]
    )
    ld.add_action(i2c_actuators_node)

    return ld
