from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
import os
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription

def generate_launch_description():
    # Include scorption_cmd_vel_mux launcer
    # scorption_cmd_vel_mux_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(FindPackageShare('scorpion_cmd_vel_mux').find('scorpion_cmd_vel_mux'), 'launch', 'cmd_vel_mux.launch.py')
    #     )
    # )

    """ Launch args """
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    use_gazebo_simulation_arg = DeclareLaunchArgument(
        "use_gazebo_simulation",
        default_value="False",
        description="Change to true to use Gazebo",
    )

    use_mujoco_simulation_arg = DeclareLaunchArgument(
        "use_mujoco_simulation",
        default_value="False",
        description="Change to true to use MuJoCo",
    )

    """ Nodes """
    # Include RVIZ launch file
    rviz_penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_rviz.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
            'use_gazebo_simulation': LaunchConfiguration('use_gazebo_simulation'),
            'use_mujoco_simulation': LaunchConfiguration('use_mujoco_simulation'),
            'joint_states_remappings': 'joint_setpoints',
        }.items()
    )

    # Include the penta_core launch file
    penta_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_core.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
            'use_gazebo_simulation': LaunchConfiguration('use_gazebo_simulation')
        }.items()
    )

    return LaunchDescription([
        # scorption_cmd_vel_mux_launch,
        name_space_arg,
        use_mujoco_simulation_arg,
        use_gazebo_simulation_arg,
        rviz_penta_pod_launch,
        penta_core_launch,
    ])
