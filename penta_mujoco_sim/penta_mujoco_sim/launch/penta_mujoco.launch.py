#!/usr/bin/env python3
"""Launch the penta robot in MuJoCo with ros2_control.

This launch file mirrors ``penta_gazebo.launch.py`` but targets the
``mujoco_ros2_control`` stack. It:

  1. Starts RViz + robot_state_publisher (via ``penta_rviz.launch.py``) with
     ``use_gazebo_simulation:=True`` so the URDF is generated with the
     simulation-time ros2_control tags. That flag also enables the MuJoCo
     ros2_control blocks (see ``use_mujoco_simulation`` xacro arg below).
  2. Runs the runtime URDF -> MJCF converter, feeding it the scene from
     ``penta_mujoco_world`` and publishing the MJCF on
     ``/mujoco_robot_description``.
  3. Runs the ``mujoco_ros2_control/ros2_control_node`` which loads the
     MJCF from that topic and drives MuJoCo.
  4. Spawns ``joint_state_broadcaster`` and ``forward_position_controller``,
     matching the Gazebo flow.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile

def generate_launch_description():
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run MuJoCo without the visualization window",
    )

    namespace_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Run a robot in MuJoCo using namespace",
    )

    penta_description_pkg = get_package_share_directory("penta_description")
    penta_pod_pkg = get_package_share_directory("penta_pod")
    penta_mujoco_world_pkg = get_package_share_directory("penta_mujoco_world")

    controllers_params_file = os.path.join(
        penta_description_pkg, "config", "ros2_control_params.yaml"
    )
    mujoco_scene_file = os.path.join(
        penta_mujoco_world_pkg, "worlds", "penta_mujoco_world.xml"
    )

    # RViz + robot_state_publisher (publishes /robot_description with
    # use_mujoco_simulation=True so the MuJoCo ros2_control blocks + the
    # <mujoco_inputs> block are present in the URDF).
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
            "joint_states_remappings": "/joint_states",
            "use_mujoco_simulation": "True",
        }.items(),
    )

    # Runtime URDF -> MJCF conversion. Reads /robot_description (published by
    # robot_state_publisher above) and publishes the MJCF on
    # /mujoco_robot_description, which the mujoco_ros2_control node reads
    # by default.
    mjcf_converter = Node(
        package="mujoco_ros2_control",
        executable="robot_description_to_mjcf.sh",
        output="both",
        emulate_tty=True,
        arguments=[
            "--scene", mujoco_scene_file,
            # Add a MuJoCo free joint at the root so the robot is not welded
            # to the world and can fall onto the floor.
            "--add_free_joint",
            "--publish_topic", "/mujoco_robot_description",
        ],
    )

    # MuJoCo ros2_control node
    mujoco_control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[
            {"use_sim_time": True},
            {"headless": LaunchConfiguration("headless")},
            ParameterFile(controllers_params_file),
        ],
        remappings=(
            [("~/robot_description", "/robot_description")]
            if os.environ.get("ROS_DISTRO") == "humble"
            else []
        ),
    )

    # ros2_control spawners (same as the Gazebo flow)
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=["joint_state_broadcaster"],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="forward_position_controller_spawner",
        output="screen",
        arguments=[
            "forward_position_controller",
            "--param-file",
            controllers_params_file,
        ],
    )

    # Bridge /joint_setpoints (sensor_msgs/JointState from the gait/kin-chain
    # pipeline) to /forward_position_controller/commands (Float64MultiArray).
    joint_setpoints_to_forward_cmd_bridge = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_to_ros2_control_command_bridge"),
                "launch",
                "gazebo_forward_joint_command_bridge.launch.py",
            )
        ),
        launch_arguments={
            "name_space": LaunchConfiguration("name_space"),
        }.items(),
    )

    return LaunchDescription(
        [
            headless_arg,
            namespace_arg,
            penta_rviz_sim_full,
            mjcf_converter,
            mujoco_control_node,
            load_joint_state_broadcaster,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=load_joint_state_broadcaster,
                    on_exit=[forward_position_controller_spawner],
                )
            ),
            joint_setpoints_to_forward_cmd_bridge,
        ]
    )
