# Import necessary modules
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def get_robot_limbs_num_from_yaml(package_name, config_folder_name, config_file_name):
    message = """
    get limbs num from inside the configuration yaml file
    """
    print(message)
    config_overall = os.path.join(
        get_package_share_directory(package_name),
        config_folder_name,
        config_file_name,
    )
    with open(config_overall, "r") as f:
        d = yaml.safe_load(f)
        limbs_num = d["/**"]["ros__parameters"]["limbs_num"]
        if type(limbs_num) != int:
            raise TypeError
    return limbs_num


def generate_launch_description():
    package_name = "penta_description"
    config_folder_name = "config"
    config_file_name = "limb_0_mdh.yaml"
    config_limb0 = os.path.join(
        get_package_share_directory(package_name),
        config_folder_name,
        config_file_name,
    )
    config_file_name = "general_config.yaml"
    limbs_num = get_robot_limbs_num_from_yaml(package_name, config_folder_name, config_file_name)
    ld = LaunchDescription()
    for i in range(limbs_num):
        limb_prefix = "limb" + str(i)
        temp_node = Node(
            package="limb_kin_chain",
            executable="limb_kin_chain_node",
            namespace=limb_prefix,
            name=limb_prefix,
            output="screen",
            parameters=[config_limb0],
            # remappings=[(individual_joint_state_topic, '/joint_states')]
        )
        ld.add_action(temp_node)  # Added the Node to LaunchDescription
    # Include the joystick teleoperation launch file
    joystick_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_teleop').find('penta_teleop'), 'launch', 'teleop_joystick.launch.py')
        )
    )
    ld.add_action(joystick_teleop_launch)
    return ld
