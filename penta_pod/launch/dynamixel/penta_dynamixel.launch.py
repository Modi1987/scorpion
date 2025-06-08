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

    robot_state_publisher_remapping_arg = DeclareLaunchArgument(
        'joint_states_remappings',
        default_value='/joint_setpoints',
        description='Robot state publisher input topic'
    )
    
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('penta_description'), 'config', 'penta.rviz'])
    rviz_config_file_arg = DeclareLaunchArgument(
            'config_file',
            default_value=rviz_config_file,
            description='Path to the RViz configuration file'
        )
    
    """ xacro args as launch arguments """
    prefix_arg = DeclareLaunchArgument(
        'prefix',
        default_value='',
        description='Prefix for the robot name space'
    )
    enable_gazebo_arg = DeclareLaunchArgument(
        'enable_gazebo',
        default_value='false',
        description='Enable Gazebo simulation, make sure to set use_real_dynamixel_motor to false when set to true'
    )
    use_real_dynamixel_motor_arg = DeclareLaunchArgument(
        'use_real_dynamixel_motor',
        default_value='true',
        description='Enable real Dynamixel motor control, make sure to set enable_gazebo to false when set to true'
    )
    port_name_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Port name for the DYNAMIXEL hardware interface, change if necessary'
    )
    
    penta_description_pkg = get_package_share_directory("penta_description")

    robot_state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(penta_description_pkg, "launch", "robot_state_publisher.launch.py")
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
            'joint_states_remappings': LaunchConfiguration('joint_states_remappings'),
            'prefix': LaunchConfiguration('prefix'),
            'enable_gazebo': LaunchConfiguration('enable_gazebo'),
            'use_real_dynamixel_motor': LaunchConfiguration('use_real_dynamixel_motor'),
            'port_name': LaunchConfiguration('port_name'),
        }.items()
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

    # Include the dynamixel control launch file
    dynamixel_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'dynamixel', 'dynamixel_control.launch.py')
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
        robot_state_publisher_remapping_arg,
        rviz_config_file_arg,
        prefix_arg,
        enable_gazebo_arg,
        use_real_dynamixel_motor_arg,
        port_name_arg,
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('config_file')]
        ),
        robot_state_publisher_launch,
        penta_pod_launch,
        gait_generator_launch,
        joints_aggregator_launch,
        null_space_publisher,
        twerk_action_server,
        dynamixel_control_launch,
        ros2_control_bridge_launch
    ])