import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import DeclareLaunchArgument
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

    log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
    )
    ld.add_action(log_level)

    """ Nodes """
    config = os.path.join(
        get_package_share_directory("imu_stabilization"),
        "config",
        "imu_stabilizer.yaml",
    )
    
    imu_stabilizer_node = launch_ros.actions.Node(
        package="imu_stabilization",
        executable="imu_stabilization_node",
        output="screen",
        namespace=LaunchConfiguration("name_space"),
        parameters=[
            {"name_space": LaunchConfiguration("name_space")},
            config
        ],
        remappings=[
            ("imu_sub", "imu/data_raw"),
            ("base_orientation_sub", "null_space_pose"),
            ("base_orientation_pub", "set_null_space_pose"),
        ],
        arguments=["--ros-args", "--log-level", LaunchConfiguration("log_level")],
    )
    ld.add_action(imu_stabilizer_node)

    return ld
