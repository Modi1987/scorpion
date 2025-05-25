from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
import os
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    """ Launch args """
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    ld.add_action(name_space_arg)

    # Launch rviz sim full
    penta_rviz_sim_full = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("penta_pod"),
                "launch",
                "penta_sim_full_rviz.launch.py",
            )
        ),
        launch_arguments={
            "name_space": LaunchConfiguration("name_space"),
        }.items(),
    )
    ld.add_action(penta_rviz_sim_full)

    # Paths
    ros_gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    # Gazebo world path
    world_pkg = get_package_share_directory("penta_gazebo_world")
    gazebo_world_path = os.path.join(
        world_pkg, "worlds", "penta_gazebo_world.world"
    )
    print(f"Gazebo world path: {gazebo_world_path}")

    # Gazebo
    ros_gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={ 'gz_args': ['-r -v4 ', gazebo_world_path], 'on_exit_shutdown': 'true' }.items()
    )
    ld.add_action(ros_gz_sim)

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_robot",
        output="screen",
        arguments=["-entity", "penta_pod", "-topic", "/robot_description"],
    )
    ld.add_action(spawn_robot)

    # ros2_control
    print(
        "ToDo: @Mohammad, check how to fix with the joint states published by the penta_pod core implementation"
    )
    load_joint_state_broadcaster = ExecuteProcess(
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
    joint_state_after_robot_spawned = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[load_joint_state_broadcaster],
        )
    )
    ld.add_action(joint_state_after_robot_spawned)

    penta_description_pkg = get_package_share_directory("penta_description")

    forward_position_controller_spawner_node = Node(
        package="controller_manager",
        executable="spawner",
        name="forward_position_controller_spawner",
        output="screen",
        arguments=[
            "forward_position_controller",
            "--param-file",
            os.path.join(penta_description_pkg, "config", "ros2_control_params.yaml"),
        ],
    )
    forward_position_controller_after_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[forward_position_controller_spawner_node],
        )
    )
    ld.add_action(forward_position_controller_after_joint_state_broadcaster)

    # Include the bridge
    set_point_to_forward_position_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_to_ros2_control_command_bridge"),
                "launch",
                "forward_joint_command_bridge.launch.py",
            )
        ),
        launch_arguments={
            "name_space": LaunchConfiguration("name_space"),
        }.items(),
    )
    ld.add_action(set_point_to_forward_position_controller)

    return ld
