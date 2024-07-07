# Import necessary modules
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("penta_pod"),
        "config",
        "limb_0.yaml",
    )
    ld = LaunchDescription()
    for i in range(5):
        limb_prefix = "limb"+str(i)
        temp_node = Node(
            package="limb_kin_chain",
            executable="limb_kin_chain_node",
            namespace=limb_prefix,
            name=limb_prefix,
            output="screen",
            parameters=[config],
            # remappings=[(individual_joint_state_topic, '/joint_states')]
        )
        ld.add_action(temp_node)  # Added the Node to LaunchDescription
    return ld
