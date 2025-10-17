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

    # Include the penta_core launch file
    penta_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_core.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )
    
    # Include the ros2 control launch file
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'ros2_control_rtrobot', 'rtrobot.ros2_control.launch.py')
        ),
    )

    # Include ros to ros2 control bridge
    ros2_control_bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('ros_to_ros2_control_command_bridge').find('ros_to_ros2_control_command_bridge'), 'launch', 'forward_joint_command_bridge.launch.py')
        ),
    )

    return LaunchDescription([
        name_space_arg,
        penta_core_launch,
        ros2_control_launch,
        ros2_control_bridge_launch
    ])