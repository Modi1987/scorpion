from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, LaunchConfiguration
from launch import LaunchDescription
import os

# This is a launch rviz2 on a laptop for the toperware_bot
def generate_launch_description():
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('penta_description'), 'config', 'penta.rviz'])
    urdf_path = os.path.join(
        get_package_share_directory("penta_description"),
        "urdf",
        "penta.urdf.xacro",
    )
    model_arg = DeclareLaunchArgument(name='model', default_value=str(urdf_path),
                                      description='Absolute path to robot urdf file')
    robot_description = ParameterValue(Command(['xacro ', LaunchConfiguration('model')]),
                                       value_type=str)
    return LaunchDescription([
        DeclareLaunchArgument(
            'config_file',
            default_value=rviz_config_file,
            description='Path to the RViz configuration file'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('config_file')]
        ),
        model_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description}
            ],
        ),
    ])