#include "penta_teleop/joystick_extra_controls.hpp"

namespace penta_pod::teleop::joystick_extra_controls {

void JoystickExtraControls::declare_parameters() {
  this->declare_base_height_parameters();
  this->declare_gait_patterns_parameters();
  this->declare_twerk_it_parameters();
}

bool JoystickExtraControls::get_parameters() {
  bool height_params_load_success = this->get_base_height_parameters();

  return height_params_load_success &&
         this->get_gait_patterns_parameters() &&
         this->get_twerk_it_parameters();
}

void JoystickExtraControls::declare_base_height_parameters() {
  node_->declare_parameter<int>("base_height_ctl.d_pad_up_down_axis_index");
  node_->declare_parameter<double>("base_height_ctl.z_base_max_value");
  node_->declare_parameter<double>("base_height_ctl.z_base_min_value");
  
}

bool JoystickExtraControls::get_base_height_parameters() {
  std::string param_name;

  param_name = "base_height_ctl.d_pad_up_down_axis_index";
  if (!node_->get_parameter(param_name, this->d_pad_up_down_axis_index_)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), this->d_pad_up_down_axis_index_);

  param_name = "base_height_ctl.z_base_max_value";
  if (!node_->get_parameter(param_name, this->z_base_max_value_)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %f",
              param_name.c_str(), this->z_base_max_value_);
  
   param_name = "base_height_ctl.z_base_min_value";
  if (!node_->get_parameter(param_name, this->z_base_min_value_)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %f",
              param_name.c_str(), this->z_base_min_value_);

  return true; // Ensure the function always returns a value
}

void JoystickExtraControls::declare_gait_patterns_parameters() {
  node_->declare_parameter<int>("gait_patterns_ctl.button_index");
}

bool JoystickExtraControls::get_gait_patterns_parameters() {
  std::string param_name;

  param_name = "gait_patterns_ctl.button_index";
  if (!node_->get_parameter(param_name, this->gait_patterns_ctl_button_index_)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), this->gait_patterns_ctl_button_index_);

  return true;
}

void JoystickExtraControls::declare_twerk_it_parameters() {
  node_->declare_parameter<int>("twerk_it.switch_mode_button_index");
  node_->declare_parameter<int>("twerk_it.trigger_button_index");
  node_->declare_parameter<double>("twerk_it.w");
  node_->declare_parameter<int>("twerk_it.dance_time_millis");

  node_->declare_parameter<std::vector<std::string>>("twerk_it.twerk_mode_names",
                                                      std::vector<std::string>{});
  node_->declare_parameter<std::vector<long int>>("twerk_it.twerk_mode_axes",
                                                      std::vector<long int>{});
  node_->declare_parameter<std::vector<double>>("twerk_it.twerk_mode_magnitudes",
                                                      std::vector<double>{});
}

bool JoystickExtraControls::get_twerk_it_parameters() {
  std::string param_name;

  param_name = "twerk_it.switch_mode_button_index";
  if (!node_->get_parameter(param_name, twerk_it_params_.switch_mode_button_index)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), twerk_it_params_.switch_mode_button_index);
  
  param_name = "twerk_it.trigger_button_index";
  if (!node_->get_parameter(param_name, twerk_it_params_.trigger_button_index)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), twerk_it_params_.trigger_button_index);

  param_name = "twerk_it.w";
  if (!node_->get_parameter(param_name, twerk_it_params_.w)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %f",
              param_name.c_str(), twerk_it_params_.w);

  param_name = "twerk_it.dance_time_millis";
  if (!node_->get_parameter(param_name, twerk_it_params_.dance_time_millis)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with value %d",
              param_name.c_str(), twerk_it_params_.dance_time_millis);

  param_name = "twerk_it.twerk_mode_names";
  if (!node_->get_parameter(param_name, twerk_it_params_.twerk_mode_names)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with %lu values",
              param_name.c_str(), twerk_it_params_.twerk_mode_names.size());

  param_name = "twerk_it.twerk_mode_axes";
  if (!node_->get_parameter(param_name, twerk_it_params_.twerk_mode_axes)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with %lu values",
              param_name.c_str(), twerk_it_params_.twerk_mode_axes.size());

  param_name = "twerk_it.twerk_mode_magnitudes";
  if (!node_->get_parameter(param_name, twerk_it_params_.magnitudes)) {
    RCLCPP_ERROR(node_->get_logger(), "Cannot load parameter %s",
                 param_name.c_str());
    return false;
  }
  RCLCPP_INFO(node_->get_logger(), "Loaded parameter %s with %lu values",
              param_name.c_str(), twerk_it_params_.magnitudes.size());

  if (twerk_it_params_.twerk_mode_names.size() !=
      twerk_it_params_.twerk_mode_axes.size() ||
      twerk_it_params_.twerk_mode_names.size() !=
      twerk_it_params_.magnitudes.size()) {
    RCLCPP_ERROR(node_->get_logger(),
                 "twerk it parameters sizes do not match. "
                 "twerk_mode_names: %lu, "
                 "twerk_mode_axes: %lu, "
                 "magnitudes: %lu",
                 twerk_it_params_.twerk_mode_names.size(),
                 twerk_it_params_.twerk_mode_axes.size(),
                 twerk_it_params_.magnitudes.size());
    return false;
  }

  return true;
}

} // namespace penta_pod::teleop::joystick_extra_controls
