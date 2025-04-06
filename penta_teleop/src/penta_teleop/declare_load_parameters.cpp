#include "penta_teleop/joystick_extra_controls.hpp"

namespace penta_pod::teleop::joystick_extra_controls {

void JoystickExtraControls::declare_parameters() {
  node_->declare_parameter<int>("base_height_ctl.d_pad_up_down_axis_index");
  node_->declare_parameter<double>("base_height_ctl.z_base_min_value");
  node_->declare_parameter<double>("base_height_ctl.z_base_max_value");
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

  return true;
}

} // namespace penta_pod::teleop::joystick_extra_controls
