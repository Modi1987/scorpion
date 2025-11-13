from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition
import os

def generate_launch_description():
    # Declare the 'mode' argument with a default value of 'real'
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Mode to run the actuators (real or sim)'
    )
    mode = LaunchConfiguration('mode')

    motors_interface_arg = DeclareLaunchArgument(
        'motors_interface',
        default_value='i2c',
        description='Type of motors to use (i2c or hiwonder)'
    )
    motors_interface = LaunchConfiguration('motors_interface')

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

    # Include the penta_i2c_actuators launch file with the mode argument (when motors_interface != hiwonder)
    penta_i2c_actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_i2c_actuators').find('penta_i2c_actuators'), 'launch', 'penta_i2c_actuators.launch.py')
        ),
        launch_arguments={'mode': mode}.items(),  # Pass the mode argument
        condition=IfCondition(EqualsSubstitution(motors_interface, 'i2c'))
    )

    # Include the penta_hiwonder_actuators launch file with the mode argument (when motors_interface == hiwonder)
    penta_hiwonder_actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_hiwonder_actuators').find('penta_hiwonder_actuators'), 'launch', 'penta_hiwonder_actuators.launch.py')
        ),
        launch_arguments={'mode': mode}.items(),  # Pass the mode argument
        condition=IfCondition(EqualsSubstitution(motors_interface, 'hiwonder'))
    )

    # Include the null space cmd publisher launch file
    null_space_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'null_pose_publisher.launch.py')
        )
    )

    # Include the twerk action server launch file
    twerk_action_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'base_twerk_action_server.launch.py')
        )
    )

    return LaunchDescription([
        mode_arg,
        motors_interface_arg,
        penta_pod_launch,
        gait_generator_launch,
        joints_aggregator_launch,
        penta_i2c_actuators_launch,
        penta_hiwonder_actuators_launch,
        null_space_publisher,
        twerk_action_server
    ])
