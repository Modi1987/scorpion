from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, EqualsSubstitution
from launch.conditions import IfCondition, UnlessCondition
import os

def generate_launch_description():
    ld = LaunchDescription()
    # Declare the 'mode' argument with a default value of 'real'
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='real',
        description='Mode to run the actuators (real or sim)'
    )
    mode = LaunchConfiguration('mode')
    ld.add_action(mode_arg)

    motors_interface_arg = DeclareLaunchArgument(
        'motors_interface',
        default_value='i2c',
        description='Type of motors to use (i2c or hiwonder)'
    )
    motors_interface = LaunchConfiguration('motors_interface')
    ld.add_action(motors_interface_arg)

    # Include the penta_pod launch file
    penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_pod.launch.py')
        )
    )
    ld.add_action(penta_pod_launch)
    
    # Include the gait_generator launch file
    gait_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gait_generator').find('gait_generator'), 'launch', 'gait_generator.launch.py')
        )
    )
    ld.add_action(gait_generator_launch)

    # Include the joints_aggregator launch file
    joints_aggregator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('joints_aggregator').find('joints_aggregator'), 'launch', 'joints_aggregator.launch.py')
        )
    )
    ld.add_action(joints_aggregator_launch)

    # Include the penta_hiwonder_actuators launch file with the mode argument (when motors_interface == hiwonder)
    penta_hiwonder_actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_hiwonder_actuators').find('penta_hiwonder_actuators'), 'launch', 'penta_hiwonder_actuators.launch.py')
        ),
        launch_arguments={'mode': mode}.items(),  # Pass the mode argument
        condition=IfCondition(EqualsSubstitution(motors_interface, 'hiwonder'))
    )
    ld.add_action(penta_hiwonder_actuators_launch)

    # Include the penta_i2c_actuators launch file with the mode argument (when motors_interface != hiwonder)
    penta_i2c_actuators_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_i2c_actuators').find('penta_i2c_actuators'), 'launch', 'penta_i2c_actuators.launch.py')
        ),
        launch_arguments={'mode': mode}.items(),  # Pass the mode argument
        condition=IfCondition(EqualsSubstitution(motors_interface, 'i2c'))
    )
    ld.add_action(penta_i2c_actuators_launch)

    # Include the null space cmd publisher launch file
    null_space_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'null_pose_publisher.launch.py')
        )
    )
    ld.add_action(null_space_publisher)

    # Include the twerk action server launch file
    twerk_action_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'base_twerk_action_server.launch.py')
        )
    )
    ld.add_action(twerk_action_server)

    return ld
