from launch import LaunchDescription
from launch.actions import RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    use_gazebo_simulation_arg = DeclareLaunchArgument(
        "use_gazebo_simulation",
        default_value="False",
        description="Enable Gazebo simulation (inside xacro), make sure to set use_real_dynamixel_motor to false when set to true",
    )

    use_rtrobot_ros2_control_arg = DeclareLaunchArgument(
        "use_rtrobot_ros2_control",
        default_value="True",
        description="Use rtrobot_ros2_control for controlling the Rtrobot servo controller",
    )

    urdf_path = os.path.join(
        get_package_share_directory("penta_description"),
        "urdf",
        "penta.urdf.xacro",
    )

    urdf_path_arg = DeclareLaunchArgument(
        name="model",
        default_value=str(urdf_path),
        description="Absolute path to robot urdf file",
    )

    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("model"),
                " use_gazebo_simulation:=",
                LaunchConfiguration("use_gazebo_simulation"),
                " use_rtrobot_ros2_control:=",
                LaunchConfiguration("use_rtrobot_ros2_control"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=LaunchConfiguration("name_space"),
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    controller_manager_config = PathJoinSubstitution(
        [
            FindPackageShare("penta_description"),
            "config",
            "rtrobot_ros2_control_params.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controller_manager_config],
        namespace=LaunchConfiguration("name_space"),
        remappings=[
            ("controller_manager/robot_description", "robot_description"),
            ("joint_states", "actuator_states"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name_space"),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
        output="screen",
    )

    forward_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace=LaunchConfiguration("name_space"),
        arguments=[
            "forward_position_controller",
            "--controller-manager",
            "controller_manager",
        ],
        output="screen",
    )

    delay_forward_controller_spawner_after_joint_state_broadcaster_spawner = (
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[forward_controller_spawner],
            )
        )
    )

    args = [
        use_gazebo_simulation_arg,
        use_rtrobot_ros2_control_arg,
        name_space_arg,
        urdf_path_arg,
    ]
    nodes = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_forward_controller_spawner_after_joint_state_broadcaster_spawner,
    ]
    return LaunchDescription(args + nodes)
