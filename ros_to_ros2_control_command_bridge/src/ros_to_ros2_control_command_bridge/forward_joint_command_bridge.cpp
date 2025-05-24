#include "ros_to_ros2_control_command_bridge/forward_joint_command_bridge.hpp"

namespace ros2_to_ros2_control_bridge
{
GazeboForwardJointCommandControlBridge::GazeboForwardJointCommandControlBridge() : 
    node_(rclcpp::Node::make_shared("gazebo_forward_joint_command_control_bridge")) {
    joint_command_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("forward_position_controller/commands", 10);
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_setpoints", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointStateCallback(msg);
        });
}

GazeboForwardJointCommandControlBridge::~GazeboForwardJointCommandControlBridge()
{
    // Destructor logic if needed
}

void GazeboForwardJointCommandControlBridge::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Process the joint state message and prepare commands
    std_msgs::msg::Float64MultiArray joint_commands;
    joint_commands.data = msg->position;  // Example: forwarding position as command

    // Publish the joint commands
    joint_command_publisher_->publish(joint_commands);
}

}