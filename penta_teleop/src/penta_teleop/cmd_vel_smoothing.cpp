#include "penta_teleop/cmd_vel_smoothing.hpp"

namespace penta_pod::teleop::twist_smoothing
{

CmdVelSmoothing::CmdVelSmoothing(rclcpp::Node::SharedPtr node)
    : node_(node) {
    RCLCPP_INFO(node_->get_logger(), "CmdVelSmoothing starting");
    // Log intracomm status
    const auto & opts = node_->get_node_options();
    if (opts.use_intra_process_comms()) {
        RCLCPP_INFO(node_->get_logger(), ">> Intra-process comms is ENABLED");
    } else {
        RCLCPP_INFO(node_->get_logger(), ">> Intra-process comms is DISABLED");
    }
    // Parameters for maximum acceleration
    declare_parameters();
    load_parameters();
    smoothed_cmd_vel_ = std::make_shared<Twist>();
    last_cmd_vel_time_ = node_->now().seconds();

    // Subscriber for raw cmd_vel
    cmd_vel_sub_ = node_->create_subscription<Twist>(
        "cmd_vel", 10,
        [this](const Twist::SharedPtr msg) { cmdVelCallback(msg); });

    // Publisher for smoothed cmd_vel
    smoothed_cmd_vel_pub_ = node_->create_publisher<Twist>(
        "smoothed_cmd_vel", 10);

    // Timer for periodic smoothing (if needed)
    auto timer_interval = static_cast<int>(1000.0 / params_.publishing_rate);
    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(timer_interval),
        [this]() {
            // Check for timeout if new target is not received
            if (node_->now().seconds() - last_cmd_vel_time_ > params_.input_cmd_vel_timeout_sec) {
                target_cmd_vel_ = std::make_shared<Twist>(); // zero twist on timeout
            }
            // Filter and publish the smoothed cmd_vel
            if (target_cmd_vel_) {
                double timer_interval = static_cast<int>(1000.0 / params_.publishing_rate);
                smoothed_cmd_vel_ = smoothenTwist(*target_cmd_vel_, *smoothed_cmd_vel_, timer_interval / 1000.0);
            }
            smoothed_cmd_vel_pub_->publish(*smoothed_cmd_vel_);
         });
}


void CmdVelSmoothing::cmdVelCallback(const Twist::SharedPtr msg)
{
    last_cmd_vel_time_ = node_->now().seconds();
    target_cmd_vel_ = msg;
}

double CmdVelSmoothing::smoothenScalar(double target, double current, double acc, double timer_interval_sec) {
    double max_step = acc * timer_interval_sec;
    double diff = target - current;
    if (std::abs(diff) > max_step) {
        diff = (diff > 0 ? max_step : -max_step);
    }
    return current + diff;
}

auto CmdVelSmoothing::smoothenTwist(const Twist& target_cmd_vel, const Twist& current_cmd_vel, double timer_interval_sec) -> Twist::SharedPtr {

    double dt = timer_interval_sec;

    Twist::SharedPtr smoothed_cmd_vel = std::make_shared<Twist>();

    // Smooth linear.x
    smoothed_cmd_vel->linear.x = smoothenScalar(target_cmd_vel.linear.x, current_cmd_vel.linear.x, params_.max_linear_acceleration, dt);
    // Smooth linear.y
    smoothed_cmd_vel->linear.y = smoothenScalar(target_cmd_vel.linear.y, current_cmd_vel.linear.y, params_.max_linear_acceleration, dt);

    // Smooth angular.z
    smoothed_cmd_vel->angular.z = smoothenScalar(target_cmd_vel.angular.z, current_cmd_vel.angular.z, params_.max_angular_acceleration, dt);

    return smoothed_cmd_vel;
}

void CmdVelSmoothing::declare_parameters() {
    node_->declare_parameter<double>("publishing_rate");
    node_->declare_parameter<double>("max_linear_acceleration");
    node_->declare_parameter<double>("max_angular_acceleration");
    node_->declare_parameter<double>("input_cmd_vel_timeout_sec");
}

void CmdVelSmoothing::load_parameters() {
    double publishing_rate_default = 10.0;
    if (!node_->get_parameter<double>("publishing_rate", publishing_rate_default)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load 'publishing_rate', using default: %f", publishing_rate_default);
    } else {
        params_.publishing_rate = publishing_rate_default;
    }

    double max_linear_acceleration_default = 0.5;
    if (!node_->get_parameter<double>("max_linear_acceleration", max_linear_acceleration_default)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load 'max_linear_acceleration', using default: %f", max_linear_acceleration_default);
    } else {
        params_.max_linear_acceleration = max_linear_acceleration_default;
    }

    double max_angular_acceleration_default = 1.0;
    if (!node_->get_parameter<double>("max_angular_acceleration", max_angular_acceleration_default)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load 'max_angular_acceleration', using default: %f", max_angular_acceleration_default);
    } else {
        params_.max_angular_acceleration = max_angular_acceleration_default;
    }

    double input_cmd_vel_timeout_sec_default = 0.2;
    if (!node_->get_parameter<double>("input_cmd_vel_timeout_sec", input_cmd_vel_timeout_sec_default)) {
        RCLCPP_WARN(node_->get_logger(), "Failed to load 'input_cmd_vel_timeout_sec', using default: %f", input_cmd_vel_timeout_sec_default);
    } else {
        params_.input_cmd_vel_timeout_sec = input_cmd_vel_timeout_sec_default;
    }
}

} // namespace penta_pod::teleop::twist_smoothing