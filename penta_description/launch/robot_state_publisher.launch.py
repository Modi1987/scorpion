from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription

def generate_launch_description():

    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )
    remapping_arg = DeclareLaunchArgument(
        'joint_states_remappings',
        default_value='/joint_states',
        description='Robot state publisher input topic'
    )

    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('penta_description'),
            'urdf',
            'penta.urdf.xacro'
        ]),
        ' name_space:=', LaunchConfiguration('name_space')
    ])

    return LaunchDescription([
        name_space_arg,
        remapping_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration('name_space'),
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description_content}
            ],
            remappings=[
                ('joint_states', LaunchConfiguration('joint_states_remappings')),
            ]
        ),
    ])