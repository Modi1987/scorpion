#include "ros_to_ros2_control_command_bridge/gazebo_forward_joint_command_bridge.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto bridge = std::make_shared<gazebo_ros2_to_ros2_control_bridge::GazeboForwardJointCommandControlBridge>();
    bridge->spin();
    rclcpp::shutdown();
    return 0;
}