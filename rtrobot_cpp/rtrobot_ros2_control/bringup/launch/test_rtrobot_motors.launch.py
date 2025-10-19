from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_rtrobot_ros2_control",
            default_value="true",
            description="Start robot with RtRobot ros2_control hardware interface.",
        )
    )

    # Initialize Arguments
    use_rtrobot_ros2_control = LaunchConfiguration("use_rtrobot_ros2_control")

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("penta_description"), "urdf", "penta.urdf.xacro"]
            ),
            " ",
            "use_rtrobot_ros2_control:=",
            use_rtrobot_ros2_control,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("penta_description"),
            "config",
            "ros2_control_params.yaml",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_controllers],
        output="both",
        remappings=[
            ("~/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    joints_forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "test_one_motor_controller",
            "--param-file",
            robot_controllers,
        ],
        # condition=IfCondition(use_mock_hardware),
    )


    nodes = [
        robot_state_pub_node,
        control_node,
        joint_state_broadcaster_spawner,
        joints_forward_position_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)
