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
import subprocess

def load_penta_pod_urdf():
    pkg_share = get_package_share_directory('penta_description')
    xacro_path = os.path.join(
        pkg_share,
        'urdf',
        'penta.urdf.xacro'
    )
    xacro_cmd = ['xacro', xacro_path]
    completed_process = subprocess.run(xacro_cmd, text=True, capture_output=True)
    if completed_process.returncode != 0:
        raise RuntimeError(f"Command '{' '.join(xacro_cmd)}' failed with error code {completed_process.returncode}")
    urdf_text = completed_process.stdout
    hard_path = f"file://{pkg_share}"
    urdf_remove_relatvie_path = urdf_text.replace('package://penta_description', hard_path)
    return urdf_remove_relatvie_path 


# This is a launch rviz2 on a laptop for the toperware_bot
def generate_launch_description():
    robot_description = load_penta_pod_urdf()
    print("robot_description: ", robot_description)
    remapping_arg = DeclareLaunchArgument(
        'joint_states_remappings',
        default_value='/joint_states',
        description='Robot state publisher input topic'
    )
    return LaunchDescription([
        remapping_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description}
            ],
            remappings=[
                ('joint_states', LaunchConfiguration('joint_states_remappings')),
            ]
        ),
    ])