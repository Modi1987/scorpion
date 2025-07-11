#include "pca9685_ros2_control/pca9685_hw_interface.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <vector>

#include "hardware_interface/lexical_casts.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#include "math.h"
#include <algorithm> // for std::transform

#include "pca9685_ros2_control/utils.hpp"

namespace pca9685_ros2_control
{
hardware_interface::CallbackReturn Pca9685HardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("pca9685_ros2_control"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Retrieve hardware parameters of Dynamixel motors
  i2c_device_name_ = info_.hardware_parameters.at("device_name");
  RCLCPP_INFO_STREAM(get_logger(), "HW Parameter i2c device_name: " << i2c_device_name_);
  
  i2c_address_ = std::stoi(info_.hardware_parameters.at("i2c_address"), nullptr, 16);
  RCLCPP_INFO_STREAM(get_logger(), "HW Parameter i2c_address: " << std::hex << i2c_address_);
  
  number_of_motors_ = std::stoi(info_.hardware_parameters["motors.number_of_channels"]);
  RCLCPP_INFO(get_logger(), "HW Parameter motors.number_of_channels: %d", number_of_motors_);

  std::string min_pulse_string  = info_.hardware_parameters.at("motors.min_pulse_width_for_each_channel_microsec");
  auto min_pulse = parse_list<long>(min_pulse_string);
  std::string max_pulse_string  = info_.hardware_parameters.at("motors.max_pulse_width_for_each_channel_microsec");
  auto max_pulse = parse_list<long>(max_pulse_string);
  std::string min_angle_string  = info_.hardware_parameters.at("motors.min_joint_angle_for_each_channel");
  auto min_angle = parse_list<long>(min_angle_string);
  std::string max_angle_string  = info_.hardware_parameters.at("motors.max_joint_angle_for_each_channel");
  auto max_angle = parse_list<long>(max_angle_string);

  if (min_pulse.size() != static_cast<size_t>(number_of_motors_) ||
      max_pulse.size() != static_cast<size_t>(number_of_motors_) ||
      min_angle.size() != static_cast<size_t>(number_of_motors_) ||
      max_angle.size() != static_cast<size_t>(number_of_motors_))
  {
    RCLCPP_FATAL(
      get_logger(), "Number of motors (%d) does not match size of pulse or angle parameters.",
      number_of_motors_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  motor_params_list_ = std::vector<MotorParams>();
  for (int i = 0; i < number_of_motors_; ++i) {
    MotorParams params{};
    params.name = "pwd_motor_" + std::to_string(i);
    params.pwm_channel = static_cast<int>(i);
    params.min_pulse = min_pulse[i] / 1000.0; // Convert to milliseconds
    params.max_pulse = max_pulse[i] / 1000.0; // Convert to milliseconds
    params.min_angle = min_angle[i];
    params.max_angle = max_angle[i];
    motor_params_list_.push_back(params);
  }

  // Initialize vectors for positions, velocities, efforts and commands
  joint_position_commands_ = std::vector<double>(number_of_motors_, 0.);
  joint_positions_ = std::vector<double>(number_of_motors_, 0.);
  joint_velocities_ = std::vector<double>(number_of_motors_, 0.);
  joint_efforts_ = std::vector<double>(number_of_motors_, 0.);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // pwm-servo control in position mode
    // - We define one command interface for position control
    // - has exactly three state interfaces: position, velocity and effort
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3)
    {
      RCLCPP_FATAL(
        get_logger(), "Joint '%s' has %zu state interface. 3 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    std::vector<std::string> expected_state_interfaces = {
      hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY,
      hardware_interface::HW_IF_EFFORT};

    for (size_t i = 0; i < joint.state_interfaces.size(); ++i)
    {
      if (joint.state_interfaces[i].name != expected_state_interfaces[i])
      {
        RCLCPP_FATAL(
          get_logger(), "Joint '%s' have '%s' as state interface. '%s' expected.",
          joint.name.c_str(), joint.state_interfaces[i].name.c_str(),
          expected_state_interfaces[i].c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685HardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &joint_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &joint_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685HardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &joint_position_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685HardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for(const auto & param : motor_params_list_) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Motor: " << param.name
      << ", Channel: " << param.pwm_channel
      << ", Min Pulse: " << param.min_pulse
      << " ms, Max Pulse: " << param.max_pulse
      << " ms, Min Angle: " << param.min_angle
      << " deg, Max Angle: " << param.max_angle
      << " deg");
  }

  pca_api_ = std::make_shared<MyPCA9685>(motor_params_list_, i2c_device_name_, i2c_address_);

  if (!pca_api_->init()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize PCA9685. Check your I2C connection and parameters.");
        rclcpp::shutdown();
        return hardware_interface::CallbackReturn::ERROR;
    }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn Pca9685HardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // shutdown the PCA9685 API
  if (pca_api_) {
    pca_api_->close();
    pca_api_.reset();
    RCLCPP_INFO(get_logger(), "PCA9685 API closed successfully.");
  } else {
    RCLCPP_WARN(get_logger(), "PCA9685 API was not initialized, nothing to close.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685HardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  static std::vector<double> previous_joint_position_commands_;
  // forward positions
  for (int index = 0; index < number_of_motors_; ++index) {
      // read position commands from the motors
      joint_positions_[index] = joint_position_commands_[index];
  }
  // defferintiate velocities
  if (previous_joint_position_commands_.empty()) {
    previous_joint_position_commands_ = joint_position_commands_;
  } else {
    for (int index = 0; index < number_of_motors_; ++index) {
      joint_velocities_[index] = (joint_position_commands_[index] - previous_joint_position_commands_[index]) / period.seconds();
      previous_joint_position_commands_[index] = joint_position_commands_[index];
    }
  }
  // efforst stays zero
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Pca9685HardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // write position commands to the motors
  for (int index = 0; index < number_of_motors_; ++index) {
      float degree = joint_position_commands_[index];
      pca_api_->setMotorCommand(index, degree); 
  }            
  pca_api_->flushInternalCommands2Motors();
  
  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  pca9685_ros2_control::Pca9685HardwareInterface, 
  hardware_interface::SystemInterface)
