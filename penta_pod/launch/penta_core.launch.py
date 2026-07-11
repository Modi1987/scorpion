from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os

def generate_launch_description():

    """ Launch args """
    # Declare the 'name_space' argument with a default value of ''
    name_space_arg = DeclareLaunchArgument(
        "name_space",
        default_value="",
        description="Robot name space",
    )

    real_hardware_arg = DeclareLaunchArgument(
        "real_hardware",
        default_value="false",
        description="Whether to use real hardware or simulation",
    )

    # Include the penta_pod launch file
    penta_pod_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('penta_pod').find('penta_pod'), 'launch', 'penta_pod.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )
    
    # Include the gait_generator launch file
    gait_generator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('gait_generator').find('gait_generator'), 'launch', 'gait_generator.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    # Include the joints_aggregator launch file
    joints_aggregator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('joints_aggregator').find('joints_aggregator'), 'launch', 'joints_aggregator.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    # Include the null space cmd publisher launch file
    null_space_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'null_pose_publisher.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    # Include the twerk action server launch file
    twerk_action_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('base_twerk').find('base_twerk'), 'launch', 'base_twerk_action_server.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    try:
        imu_pkg_share = get_package_share_directory("imu_arduino_serial_ros2_interface")
        print("imu_arduino_serial_ros2_interface is available will launch imu_node.launch.py")
        imu_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(imu_pkg_share, 'launch', 'imu_node.launch.py')
            ),
            launch_arguments={
                'name_space': LaunchConfiguration('name_space'),
                'real_hardware': LaunchConfiguration('real_hardware'),
            }.items()
        )
    except PackageNotFoundError:
        print("imu_arduino_serial_ros2_interface is not available, reverting to pentapod_imu package")
        imu_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(FindPackageShare('pentapod_imu').find('pentapod_imu'), 'launch', 'pentapod_imu.launch.py')
            ),
            launch_arguments={
                'name_space': LaunchConfiguration('name_space'),
                'real_hardware': LaunchConfiguration('real_hardware'),
            }.items()
        )

    imu_stabilizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(FindPackageShare('imu_stabilization').find('imu_stabilization'), 'launch', 'imu_stabilizer.launch.py')
        ),
        launch_arguments={
            'name_space': LaunchConfiguration('name_space'),
        }.items()
    )

    return LaunchDescription([
        name_space_arg,
        real_hardware_arg,
        penta_pod_launch, # for limbs and joystick
        gait_generator_launch,
        joints_aggregator_launch,
        null_space_publisher,
        twerk_action_server,
        imu_launch,
        imu_stabilizer_launch,
    ])
