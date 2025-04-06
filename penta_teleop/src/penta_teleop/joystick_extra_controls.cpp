#include "penta_teleop/joystick_extra_controls.hpp"

namespace penta_pod::teleop::joystick_extra_controls {

JoystickExtraControls::JoystickExtraControls()
    : node_{rclcpp::Node::make_shared("joystick_extra_controls")} {
  RCLCPP_INFO(node_->get_logger(),
              "Starting joystick_extra_controls, subscribing to joy topic and "
              "calling base_pose_setpoint service");
  // load parameters
  this->declare_parameters();
  this->get_parameters();

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  // Joystick subscriber
  joy_subscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        this->joy_sub_callback(msg);
      });

  // Create service clients
  get_current_base_pose_client_ = node_->create_client<GetCurrentBasePose>(
      "get_current_null_pose", rmw_qos_profile_services_default,
      callback_group_);
  set_base_pose_client_ = node_->create_client<BasePoseSetpoint>(
      "cmd_null_setpoint", rmw_qos_profile_services_default, callback_group_);
}

void JoystickExtraControls::joy_sub_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {

  if (msg->axes[this->d_pad_up_down_axis_index_] != 0) {
    double delta_z = 0.001;
    double z_disp_base_command = msg->axes[7] * delta_z;

    // Check if the service is available before calling
    if (!get_current_base_pose_client_->wait_for_service(
            std::chrono::seconds(100))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Service get_current_base_pose is unavailable.");
      return;
    }

    // Prepare the request
    auto request = std::make_shared<GetCurrentBasePose::Request>();

    // Call the service asynchronously with a callback
    auto future = get_current_base_pose_client_->async_send_request(
        request,
        [this, z_disp_base_command](
            rclcpp::Client<GetCurrentBasePose>::SharedFuture response) {
          this->handle_get_base_pose_response(response, z_disp_base_command);
        });
  }
}

void JoystickExtraControls::handle_get_base_pose_response(
    rclcpp::Client<GetCurrentBasePose>::SharedFuture response,
    double z_disp_base_command) {

  auto result = response.get();

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get base pose.");
    return;
  }

  // Sanity checks
  if ((result->pose.pose.position.z > this->z_base_max_value_) &&
      z_disp_base_command > 0.) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Already above the max Z limit, can not move up any further.");
    return;
  }

  if ((result->pose.pose.position.z < this->z_base_min_value_) &&
      z_disp_base_command < 0.) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Already below the min Z limit, can not move down any further.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Current base pose: %f %f %f %f %f %f %f",
              result->pose.pose.position.x, result->pose.pose.position.y,
              result->pose.pose.position.z, result->pose.pose.orientation.x,
              result->pose.pose.orientation.y, result->pose.pose.orientation.z,
              result->pose.pose.orientation.w);

  // Prepare new base pose setpoint request
  auto setpoint_request = std::make_shared<BasePoseSetpoint::Request>();
  setpoint_request->pose = result->pose;
  setpoint_request->pose.pose.position.z += z_disp_base_command;

  // Call the set_base_pose service asynchronously
  auto future = set_base_pose_client_->async_send_request(
      setpoint_request,
      [this](rclcpp::Client<BasePoseSetpoint>::SharedFuture setpoint_response) {
        this->handle_set_base_pose_response(setpoint_response);
      });
}

void JoystickExtraControls::handle_set_base_pose_response(
    rclcpp::Client<BasePoseSetpoint>::SharedFuture response) {

  auto result = response.get();

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set base pose.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Base pose set successfully.");
  }
}

} // namespace penta_pod::teleop::joystick_extra_controls
