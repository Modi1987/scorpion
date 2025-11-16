from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    """Launch args"""
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    name_space = LaunchConfiguration("name_space")

    real_hardware_arg = DeclareLaunchArgument(
        "real_hardware",
        default_value="true",
        description="Whether to use real hardware or simulation",
    )
    real_hardware_flag = LaunchConfiguration("real_hardware")

    args = [name_space_arg, real_hardware_arg]

    """ Nodes """
    pkg_share = FindPackageShare(package="pentapod_imu").find("pentapod_imu")
    config_file = os.path.join(pkg_share, "config", "pentapod_imu_config.yaml")
    imu_node = Node(
        package="pentapod_imu",
        executable="pentapod_imu_node",
        namespace=name_space,
        parameters=[config_file],
        output="both",
        remappings=[
            ("imu/data_raw", "imu/data_raw"),
        ],
    )

    nodes = [
        imu_node,
    ]

    return LaunchDescription(args + nodes)
