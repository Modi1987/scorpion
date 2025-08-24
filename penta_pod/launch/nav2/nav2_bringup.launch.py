import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


# This is a launch file used to launch slam nodes on laptop
def generate_launch_description():
    # name space argument
    declare_name_space_argument = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    # use gazebo simulation argument
    use_gazebo_simulation_arg = DeclareLaunchArgument(
        "use_gazebo_simulation",
        default_value="False",
        description="Change to true to use Gazebo",
    )
    # start utils nodes
    utils_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("penta_pod"),
                    "launch/nav2/nav2_util_nodes.launch.py",
                )
            ]
        )
    )
    # start slam
    use_sim_time = LaunchConfiguration("use_sim_time")
    scan_topic = [LaunchConfiguration("name_space"), "/scan"]
    map_frame = [LaunchConfiguration("name_space"), "map"]
    base_frame = [LaunchConfiguration("name_space"), "base_footprint"]
    odom_frame = [LaunchConfiguration("name_space"), "odom"]
    slam_params_file = LaunchConfiguration("slam_params_file")

    declare_use_sim_time_argument = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use simulation/Gazebo clock"
    )
    declare_slam_params_file_cmd = DeclareLaunchArgument(
        "slam_params_file",
        default_value=os.path.join(
            get_package_share_directory("penta_pod"),  # "slam_toolbox"),
            "config",
            "mapper_params_online_async.yaml",
        ),
        description="Full path to the ROS2 parameters file to use for the slam_toolbox node",
    )

    start_async_slam_toolbox_node = Node(
        parameters=[
            slam_params_file,
            {"use_sim_time": use_sim_time},
            {"odom_frame": odom_frame},
            {"map_frame": map_frame},
            {"base_frame": base_frame},
            {"scan_topic": scan_topic},
            {"use_map_saver": True},
            {"mode": "mapping"},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "launch/navigation_launch.py",
                )
            ]
        )
    )

    # Include RVIZ launch file
    rviz_penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_rviz.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
            'use_gazebo_simulation': LaunchConfiguration('use_gazebo_simulation'),
            'joint_states_remappings': 'joint_setpoints',
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(declare_name_space_argument)
    ld.add_action(use_gazebo_simulation_arg)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_slam_params_file_cmd)
    ld.add_action(utils_launch)
    ld.add_action(start_async_slam_toolbox_node)
    ld.add_action(nav2_launch)
    ld.add_action(rviz_penta_pod_launch)

    return ld
