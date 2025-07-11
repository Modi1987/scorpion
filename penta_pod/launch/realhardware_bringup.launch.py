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

    # Include the penta_core launch file
    penta_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_core.launch.py')
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

    return LaunchDescription([
        name_space_arg,
        mode_arg,
        penta_core_launch,
        penta_i2c_actuators_launch,
    ])
