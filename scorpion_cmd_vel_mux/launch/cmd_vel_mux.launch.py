import os

import ament_index_python.packages
import launch
import launch_ros.actions

import yaml


def generate_launch_description():
    share_dir = ament_index_python.packages.get_package_share_directory('scorpion_cmd_vel_mux')
    params_file = os.path.join(share_dir, 'config', 'cmd_vel_mux_params.yaml')
    with open(params_file, 'r') as f:
        params = yaml.safe_load(f)['scorpion_cmd_vel_mux']['ros__parameters']

    cmd_vel_mux_node = launch_ros.actions.Node(
        package='cmd_vel_mux',
        executable='cmd_vel_mux_node',
        output='both',
        parameters=[params],
        # remappings={("/cmd_vel", "output/cmd_vel_ref"), # this is for the output topic
        #             ("input/cmd_vel_nav2", "/cmd_vel")} # this is for the input from nav2
    )

    return launch.LaunchDescription([cmd_vel_mux_node])
