from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

def generate_launch_description():

    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    
    return LaunchDescription([
        name_space_arg,
        Node(
            package='laser_scan_remapper',
            executable='laser_scan_remapper_node',
            namespace=LaunchConfiguration('name_space'),
            name='laser_scan_remapper_node',
            output='screen',
            parameters=[
                {'name_space': LaunchConfiguration('name_space')}
            ],
            remappings=[
                # remapping of input topic
                ('gz/scan', 'gz/scan'),
                # remapping of output topic
                ('scan', 'scan')
            ]
        ),
    ])