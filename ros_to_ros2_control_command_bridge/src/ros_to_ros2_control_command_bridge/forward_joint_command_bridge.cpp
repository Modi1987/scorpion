#include "ros_to_ros2_control_command_bridge/forward_joint_command_bridge.hpp"

namespace ros2_to_ros2_control_bridge
{
ForwardJointCommandControlBridge::ForwardJointCommandControlBridge() : 
    node_(rclcpp::Node::make_shared("forward_joint_command_control_bridge")) {
    // Initialize the node and declare parameters
    RCLCPP_INFO(node_->get_logger(), "ForwardJointCommandControlBridge initialized.");
    declareParameters();
    loadParameters();
    // Create the publisher and subscription
    joint_command_publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("actuator_setpoints", 10);
    joint_state_subscription_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_setpoints", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            this->jointStateCallback(msg);
        });
}

ForwardJointCommandControlBridge::~ForwardJointCommandControlBridge()
{
    // Destructor logic if needed
}

void ForwardJointCommandControlBridge::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    // Process the joint state message and prepare commands
    std_msgs::msg::Float64MultiArray joint_commands;
    joint_commands.data = msg->position;  // Example: forwarding position as command
    auto n = msg->position.size();
    for (size_t i = 0; i < n; ++i) {
        auto q_i_degree = msg->position[i] * 180 / M_PI;
        auto dq_i_degree = q_i_degree - joint_angles_at_initial_pose_degree_[i];
        auto servo_setpoint_degree = actuator_angles_at_initial_pose_degree_[i] + dir_[i] * dq_i_degree;
        auto servo_setpoint_rad = servo_setpoint_degree * (M_PI / 180.0); // Convert degrees to radians
        joint_commands.data[i] = servo_setpoint_rad;
    }
    // Publish the joint commands
    joint_command_publisher_->publish(joint_commands);
}

void ForwardJointCommandControlBridge::declareParameters()
{
    node_->declare_parameter<std::vector<double>>("joint_angles_at_initial_pose_degree",
        std::vector<double>{}
    );
    node_->declare_parameter<std::vector<double>>(
        "cpp_pca9685_i2c_actuators_params.actuator_angles_at_initial_pose_degree",
        std::vector<double>()
    );
    node_->declare_parameter<std::vector<double>>("cpp_pca9685_i2c_actuators_params.dir",
        std::vector<double>()
    );
}

void ForwardJointCommandControlBridge::loadParameters()
{
    auto load_vector_parameter = [this](const std::string &param_name, std::vector<double> &param_vector) {
        if (!node_->get_parameter(param_name, param_vector)) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to load parameter: %s", param_name.c_str());
        } else {
            std::string formatted_values = "(" + param_name + ") parameter loaded and equal to: [";
            for (auto &value : param_vector)
                formatted_values += " " + std::to_string(value);
            formatted_values += " ]";
            RCLCPP_INFO(node_->get_logger(), formatted_values.c_str());
        }
    };

    load_vector_parameter("joint_angles_at_initial_pose_degree", joint_angles_at_initial_pose_degree_);
    load_vector_parameter("cpp_pca9685_i2c_actuators_params.actuator_angles_at_initial_pose_degree", actuator_angles_at_initial_pose_degree_);
    load_vector_parameter("cpp_pca9685_i2c_actuators_params.dir", dir_);
}

}