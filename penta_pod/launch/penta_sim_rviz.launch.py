from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Include RVIZ launch file
    rviz_penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_rviz.launch.py')
        ),
        launch_arguments={
            'joint_states_remappings': '/joint_setpoints',
        }.items()
    )

    # Include the penta_pod launch file
    penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_pod.launch.py')
        )
    )
    
    # Include the gait_generator launch file
    gait_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gait_generator').find('gait_generator'), 'launch', 'gait_generator.launch.py')
        )
    )

    # Include the joints_aggregator launch file
    joints_aggregator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('joints_aggregator').find('joints_aggregator'), 'launch', 'joints_aggregator.launch.py')
        )
    )

    return LaunchDescription([
        rviz_penta_pod_launch,
        penta_pod_launch,
        gait_generator_launch,
        joints_aggregator_launch,
    ])
