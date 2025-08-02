from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # name space argument
    declare_name_space_argument = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    return LaunchDescription([
        declare_name_space_argument,
        # Declare launch arguments for frame names
        DeclareLaunchArgument('parent_frame', default_value='base_link'),
        DeclareLaunchArgument('child_frame', default_value='laser'),

        # Declare launch arguments for transform (optional)
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        DeclareLaunchArgument('qx', default_value='0.0'),
        DeclareLaunchArgument('qy', default_value='0.0'),
        DeclareLaunchArgument('qz', default_value='0.0'),
        DeclareLaunchArgument('qw', default_value='1.0'),

        # Node that runs static_transform_publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=[
                LaunchConfiguration('x'),
                LaunchConfiguration('y'),
                LaunchConfiguration('z'),
                LaunchConfiguration('qx'),
                LaunchConfiguration('qy'),
                LaunchConfiguration('qz'),
                LaunchConfiguration('qw'),
                [LaunchConfiguration('name_space'), LaunchConfiguration('parent_frame')],
                [LaunchConfiguration('name_space'), LaunchConfiguration('child_frame')],
            ]
        ),
        Node(
            package="twist_to_odom",
            executable="twist_to_odom_node",
            namespace=LaunchConfiguration("name_space"),
            output="screen",
            remappings=[
                ("feedback_cmd_vel", "feedback_cmd_vel"),
                ("odom", "odom"),
            ],
        ),
    ])
