#include "penta_teleop/joystick_extra_controls.hpp"

namespace penta_pod::teleop::joystick_extra_controls {

void JoystickExtraControls::declare_parameters() {
  node_->declare_parameter<int>("base_height_ctl.d_pad_up_down_axis_index");
  node_->declare_parameter<int>("gait_patterns_ctl.button_index");
  node_->declare_parameter<int>("gait_parameters.gait_patterns.num");
  node_->declare_parameter<double>("base_height_ctl.z_base_min_value");
  node_->declare_parameter<double>("base_height_ctl.z_base_max_value");
  node_->declare_parameter<int>("shake_it.up_and_down.trigger_button_index");
  node_->declare_parameter<double>("shake_it.up_and_down.magnitude");
  node_->declare_parameter<double>("shake_it.up_and_down.w");
  node_->declare_parameter<int>("shake_it.up_and_down.dance_time_millis");
}

bool JoystickExtraControls::get_parameters() {
  std::string param_name;

  param_name = "base_height_ctl.d_pad_up_down_axis_index";
  if (!node_->get_parameter(param_name, this->d_pad_up_down_axis_index_)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), this->d_pad_up_down_axis_index_);

  param_name = "gait_patterns_ctl.button_index";
  if (!node_->get_parameter(param_name,
                            this->gait_patterns_ctl_button_index_)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), this->gait_patterns_ctl_button_index_);

  param_name = "gait_parameters.gait_patterns.num";
  if (!node_->get_parameter(param_name, this->num_of_gait_patterns_)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), this->num_of_gait_patterns_);

  param_name = "base_height_ctl.z_base_max_value";
  if (!node_->get_parameter(param_name, this->z_base_max_value_)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %f",
              param_name.c_str(), this->z_base_max_value_);

  param_name = "base_height_ctl.z_base_min_value";
  if (!node_->get_parameter(param_name, this->z_base_min_value_)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %f",
              param_name.c_str(), this->z_base_min_value_);
  
  param_name = "shake_it.up_and_down.trigger_button_index";
  if (!node_->get_parameter(param_name, this->up_and_down_shake_params_.trigger_button_index)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), this->up_and_down_shake_params_.trigger_button_index);
  
  param_name = "shake_it.up_and_down.magnitude";
  if (!node_->get_parameter(param_name, this->up_and_down_shake_params_.magnitude)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %f",
              param_name.c_str(), this->up_and_down_shake_params_.magnitude);
  
  param_name = "shake_it.up_and_down.w";
  if (!node_->get_parameter(param_name, this->up_and_down_shake_params_.w)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %f",
              param_name.c_str(), this->up_and_down_shake_params_.w);

  param_name = "shake_it.up_and_down.dance_time_millis";
  if (!node_->get_parameter(param_name, this->up_and_down_shake_params_.dance_time_millis)) {
    RCLCPP_ERROR(node_->get_logger(), "Can not load parameters %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), this->up_and_down_shake_params_.dance_time_millis);
              
  return true;
}

} // namespace penta_pod::teleop::joystick_extra_controls
