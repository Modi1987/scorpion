from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    pkg_share = FindPackageShare(package="pentapod_imu").find("pentapod_imu")
    config_file = os.path.join(pkg_share, "config", "pentapod_imu_config.yaml")
    imu_node = Node(
        package="pentapod_imu",
        executable="pentapod_imu_node",
        parameters=[config_file],
        output="both",
        remappings=[
            ("imu/data_raw", "imu/data_raw"),
        ],
    )

    nodes = [
        imu_node,
    ]

    return LaunchDescription(nodes)
