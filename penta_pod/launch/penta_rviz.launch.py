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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# This is a launch rviz2 on a laptop for the toperware_bot
def generate_launch_description():
    """ Launch args """
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    robot_state_publisher_remapping_arg = DeclareLaunchArgument(
        'joint_states_remappings',
        default_value='/joint_states',
        description='Robot state publisher input topic'
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('penta_description'), 'config', 'penta.rviz'])
    rviz_config_file_arg = DeclareLaunchArgument(
            'config_file',
            default_value=rviz_config_file,
            description='Path to the RViz configuration file'
        )
    
    penta_description_pkg = get_package_share_directory("penta_description")

    # RViz
    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(penta_description_pkg, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
            'joint_states_remappings': LaunchConfiguration('joint_states_remappings'),
        }.items()
    )
    return LaunchDescription([
        name_space_arg,
        robot_state_publisher_remapping_arg,
        rviz_config_file_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('config_file')]
        ),
        robot_state_publisher_launch,
    ])