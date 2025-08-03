from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

def generate_launch_description():
    """
    Check if rplidar_ros package is installed.
    """
    try:
        get_package_share_directory('rplidar_ros')
    except PackageNotFoundError as e:
        # Use a LogInfo action so it's visible in ros2 launch output
        return LaunchDescription([
            LogInfo(msg="RPLIDAR ROS package not found. Skipping rplidar node.")
        ])

    # Declare namespace argument
    declare_name_space_argument = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    rplidar_node = Node(
        name='rplidar_composition',
        package='rplidar_ros',
        executable='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'serial_baudrate': 115200,  # A1 / A2
            # 'serial_baudrate': 256000, # A3
            'frame_id': [LaunchConfiguration('name_space'), 'laser'],
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    return LaunchDescription([
        declare_name_space_argument,
        rplidar_node,
    ])
