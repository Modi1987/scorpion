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
    ros_gz_sim_pkg = get_package_share_directory("ros_gz_sim")

    # RViz
    penta_rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(penta_pkg, "launch", "penta_rviz.launch.py")
        ),
        launch_arguments={
            'joint_states_remappings': "/joint_states",
        }.items()
    )

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

    # Spawn robot into Gazebo
    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
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

    

    penta_description_pkg = get_package_share_directory("penta_description")

    forward_position_controller_spwaner_node = Node(
        package="controller_manager",
        executable="spawner",
        name="forward_position_controller_spawner",
        output="screen",
        arguments=["forward_position_controller", "--param-file", os.path.join(penta_description_pkg, "config", "ros2_control_params.yaml")],
    )

    # Create a node for the ROS-Gazebo bridge to handle message passing
    gz_bridge_params_path = os.path.join(
        get_package_share_directory("penta_gazebo_sim"), "config", "ros_gz_bridge_params.yaml"
    )
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output='screen'
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
            ros_gz_sim,
            spawn_robot,
            gz_bridge_node
        ]
    )
