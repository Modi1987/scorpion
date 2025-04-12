from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # RVIZ launch file
    penta_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('penta_pod'), 'launch', 'penta_sim_full_rviz.launch.py')
        )
    )
    # Gazebo ros2 node
    gazebo_ros = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )
    # Gazebo launch file
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-entity', 'penta_pod',
            '-topic', '/robot_description',
        ]
    )
    return LaunchDescription([
        penta_rviz_launch,
        gazebo_ros,
        spawn_robot
    ])
