from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )

    # Initialize Arguments
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")

    # Get URDF via xacro
    robot_description_content = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [FindPackageShare("penta_description"), "urdf", "penta.urdf.xacro"]
                ),
                " ",
                "use_mock_hardware:=",
                use_mock_hardware,
            ]
        ),
        value_type=str,
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("pca9685_hw_interface"),
            "config",
            "test_controllers.yaml",
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
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
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

    elbow_wrist_joints_controller_spwaner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "elbow_wrist_joints_controller",
            "--param-file",
            robot_controllers,
        ],
        # condition=IfCondition(use_mock_hardware),
    )

    claw_joints_controller_spwaner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "claw_joints_controller",
            "--param-file",
            robot_controllers,
        ],
        # condition=IfCondition(use_mock_hardware),
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        elbow_wrist_joints_controller_spwaner,
        claw_joints_controller_spwaner,
    ]

    return LaunchDescription(declared_arguments + nodes)
