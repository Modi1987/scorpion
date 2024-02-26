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
        get_package_share_directory("limb_kin_chain"),
        "config",
        "limb_0.yaml",
    )
    ld = LaunchDescription()
    limbs_joint_states_topics_list = []
    for i in range(4):
        limb_prefix = "limb"+str(i)
        individual_joint_state_topic = "/" + limb_prefix + "/joint_state"
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
        
        limbs_joint_states_topics_list.append(individual_joint_state_topic)
    # mapp_to = "/joint_states"
    # limbs_joint_states_topics_list.append(mapp_to)
    # joint_state_relay_node = Node(
    #     package='topic_tools',
    #     executable='relay',
    #     arguments=limbs_joint_states_topics_list,
    #     output='screen',
    # )
    # ld.add_action(joint_state_relay_node)
    return ld
