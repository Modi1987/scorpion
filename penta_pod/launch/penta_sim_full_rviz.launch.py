from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_description():
    # Include scorption_cmd_vel_mux launcer
    # scorption_cmd_vel_mux_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(FindPackageShare('scorpion_cmd_vel_mux').find('scorpion_cmd_vel_mux'), 'launch', 'cmd_vel_mux.launch.py')
    #     )
    # )

    ld = LaunchDescription()

    """ Launch args """
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    ld.add_action(name_space_arg)

    """ Nodes """
    # Include RVIZ launch file
    rviz_penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_rviz.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
            'joint_states_remappings': 'joint_setpoints',
        }.items()
    )

    # Include the penta_pod launch file (limbs)
    penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_pod.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )
    
    # Include the gait_generator launch file
    gait_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gait_generator').find('gait_generator'), 'launch', 'gait_generator.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    # Include the joints_aggregator launch file
    joints_aggregator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('joints_aggregator').find('joints_aggregator'), 'launch', 'joints_aggregator.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    # Include the null space cmd publisher launch file
    null_space_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'null_pose_publisher.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    # Include the twerk action server launch file
    twerk_action_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'base_twerk_action_server.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    return LaunchDescription([
        # scorption_cmd_vel_mux_launch,
        name_space_arg,
        rviz_penta_pod_launch,
        penta_pod_launch,
        gait_generator_launch,
        joints_aggregator_launch,
        null_space_publisher,
        twerk_action_server
    ])
