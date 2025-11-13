from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths
    penta_pkg = get_package_share_directory("penta_pod")
    gazebo_ros_pkg = get_package_share_directory("gazebo_ros")

    # RViz
    penta_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(penta_pkg, "launch", "penta_rviz.launch.py")
        ),
        launch_arguments={
            'joint_states_remappings': "/joint_states",
        }.items()
    )
    
    # Gazebo
    gazebo_ros = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, "launch", "gazebo.launch.py")
        )
    )

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_robot",
        output="screen",
        arguments=["-entity", "penta_pod", "-topic", "/robot_description"],
    )

    # ros2_control
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # load_joint_trajectory_controller = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "control",
    #         "load_controller",
    #         "--set-state",
    #         "active",
    #         "joint_trajectory_controller",
    #     ],
    #     output="screen",
    # )

    # load_forward_position_controller = ExecuteProcess(
    #     cmd=[
    #         "ros2",
    #         "run",
    #         "controller_manager",
    #         "--set-state",
    #         "active",
    #         "forward_position_controller",
    #     ],
    #     output="screen",
    # )

    penta_description_pkg = get_package_share_directory("penta_description")

    forward_position_controller_spwaner_node = Node(
        package="controller_manager",
        executable="spawner",
        name="forward_position_controller_spawner",
        output="screen",
        arguments=["forward_position_controller", "--param-file", os.path.join(penta_description_pkg, "config", "ros2_control_params.yaml")],
    )


    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=spawn_robot,
                    on_exit=[load_joint_state_controller],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_controller,
                    on_exit=[forward_position_controller_spwaner_node],
                )
            ),
            penta_rviz_launch,
            gazebo_ros,
            spawn_robot,
        ]
    )
