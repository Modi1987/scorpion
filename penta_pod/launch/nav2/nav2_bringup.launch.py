import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

import subprocess

# This is a launch file used to launch slam nodes on laptop
def generate_launch_description():
    # name space argument
    declare_name_space_argument = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    # start odometry
    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("cmd_vel_to_odom"),
                    "launch/cmd_vel_to_odom.launch.py",
                )
            ]
        )
    )
    # start slam
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic = [LaunchConfiguration('name_space'), '/scan']
    map_frame = [LaunchConfiguration('name_space'), 'map']
    base_frame = [LaunchConfiguration('name_space'), 'base_footprint']
    odom_frame = [LaunchConfiguration('name_space'), 'odom']
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(get_package_share_directory("penta_pod"), #"slam_toolbox"),
                                   'config', 'mapper_params_online_async.yaml'),
        description='Full path to the ROS2 parameters file to use for the slam_toolbox node')

    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time},
          {'odom_frame': odom_frame},
          {'map_frame': map_frame},
          {'base_frame': base_frame},
          {'scan_topic': scan_topic},
          {'use_map_saver': True},
          {'mode': 'mapping'}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')
    
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch/navigation_launch.py",
                )
            ]
        )
    )

    ld = LaunchDescription()
    ld.add_action(declare_name_space_argument)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(odometry_launch)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(nav2_launch)

    return ld
