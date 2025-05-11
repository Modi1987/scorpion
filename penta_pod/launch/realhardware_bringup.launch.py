from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():

    """ Launch args """
    # Declare the 'name_space' argument with a default value of ''
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    # Declare the 'mode' argument with a default value of 'real'
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Mode to run the actuators (real or sim)'
    )

    # Include the penta_pod launch file
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
        )
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

    # Include the penta_i2c_actuators launch file with the mode argument
    penta_i2c_actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_i2c_actuators').find('penta_i2c_actuators'), 'launch', 'penta_i2c_actuators.launch.py')
        ),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'name_space': LaunchConfiguration('name_space'),
            }.items()  # Pass the argyments
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
        name_space_arg,
        mode_arg,
        penta_pod_launch,
        gait_generator_launch,
        joints_aggregator_launch,
        penta_i2c_actuators_launch,
        null_space_publisher,
        twerk_action_server
    ])
