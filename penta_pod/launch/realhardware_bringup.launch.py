from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
import os

def generate_launch_description():

    """ Launch args """
    # Declare the 'name_space' argument with a default value of ''
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    # Declare the 'mode' argument with a default value of 'real' otherwise 'virtual'
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real', # 'virtual'
        description='Mode to run the actuators (real or sim)'
    )

    motors_interface_arg = DeclareLaunchArgument(
        'motors_interface',
        default_value='i2c',
        description='Type of motors to use (i2c or hiwonder)'
    )
    motors_interface = LaunchConfiguration('motors_interface')


    # Include the penta_core launch file
    penta_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_core.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    # Include the penta_i2c_actuators launch file with the mode argument (when motors_interface != hiwonder)
    penta_i2c_actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_i2c_actuators').find('penta_i2c_actuators'), 'launch', 'penta_i2c_actuators.launch.py')
        ),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'name_space': LaunchConfiguration('name_space'),
        }.items(),  # Pass the argyments
        condition=IfCondition(EqualsSubstitution(motors_interface, 'i2c'))
    )


    # Include the penta_hiwonder_actuators launch file with the mode argument (when motors_interface == hiwonder)
    penta_hiwonder_actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_hiwonder_actuators').find('penta_hiwonder_actuators'), 'launch', 'penta_hiwonder_actuators.launch.py')
        ),
        launch_arguments={
            'mode': LaunchConfiguration('mode'),
            'name_space': LaunchConfiguration('name_space'),
            'joints_setpoint_topic': 'joint_setpoints',
        }.items(),  # Pass the mode argument
        condition=IfCondition(EqualsSubstitution(motors_interface, 'hiwonder'))
    )

    # Include the penta_rplidar launch file
    penta_rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'nav2', 'penta_pod_rplidar.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    return LaunchDescription([
        name_space_arg,
        mode_arg,
        penta_core_launch,
        penta_rplidar_launch,
        motors_interface_arg,
        penta_i2c_actuators_launch,
        penta_hiwonder_actuators_launch,
    ])
