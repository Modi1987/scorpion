#include "rtrobot_ros2_control/rtrobot_hw_interface.hpp"

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

#include "rtrobot_ros2_control/utils.hpp"

namespace rtrobot_ros2_control
{
hardware_interface::CallbackReturn RtRobotHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  logger_ = std::make_shared<rclcpp::Logger>(
    rclcpp::get_logger("rtrobot_ros2_control"));
  clock_ = std::make_shared<rclcpp::Clock>(rclcpp::Clock());

  // Retrieve hardware parameters of Dynamixel motors
  serial_port_name_ = info_.hardware_parameters.at("device_name");
  RCLCPP_INFO_STREAM(get_logger(), "HW Parameter serial device_name: " << serial_port_name_);

  baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"), nullptr, 10);
  RCLCPP_INFO_STREAM(get_logger(), "HW Parameter baud_rate: " << baud_rate_);

  trajectory_param_ = info_.hardware_parameters.at("trajectory_param");
  RCLCPP_INFO_STREAM(get_logger(), "HW Parameter trajectory_param: " << trajectory_param_);

  number_of_motors_ = std::stoi(info_.hardware_parameters["motors.number_of_channels"]);
  RCLCPP_INFO(get_logger(), "HW Parameter motors.number_of_channels: %d", number_of_motors_);

  std::string min_pulse_string  = info_.hardware_parameters.at("motors.min_pulse_width_for_each_channel_microsec");
  auto min_pulse_micro = parse_list<long>(min_pulse_string);
  std::string max_pulse_string  = info_.hardware_parameters.at("motors.max_pulse_width_for_each_channel_microsec");
  auto max_pulse_micro = parse_list<long>(max_pulse_string);
  std::string min_angle_degree_string  = info_.hardware_parameters.at("motors.min_joint_angle_degree_for_each_channel");
  auto min_angle_degree = parse_list<long>(min_angle_degree_string);
  std::string max_angle_degree_string  = info_.hardware_parameters.at("motors.max_joint_angle_degree_for_each_channel");
  auto max_angle_degree = parse_list<long>(max_angle_degree_string);
  auto init_angles_degree = parse_list<long>(info_.hardware_parameters.at("motors.init_angles_degree"));

  if (min_pulse_micro.size() != static_cast<size_t>(number_of_motors_) ||
      max_pulse_micro.size() != static_cast<size_t>(number_of_motors_) ||
      min_angle_degree.size() != static_cast<size_t>(number_of_motors_) ||
      max_angle_degree.size() != static_cast<size_t>(number_of_motors_) ||
      init_angles_degree.size() != static_cast<size_t>(number_of_motors_))
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
    params.min_pulse_micro = min_pulse_micro[i]; // Microseconds
    params.max_pulse_micro = max_pulse_micro[i]; // Microseconds
    params.min_angle_degree = min_angle_degree[i]; // in degrees
    params.max_angle_degree = max_angle_degree[i]; // in degrees
    motor_params_list_.push_back(params);
  }

  // Initialize vectors for positions, velocities, efforts and commands
  actuator_position_commands_rad_ = std::vector<double>(number_of_motors_, 0.);
  actuator_position_feedback_rad_ = std::vector<double>(number_of_motors_, 0.);
  actuator_pwm_command_micro_sec_ = std::vector<int>(number_of_motors_, 0);
  for (int i = 0; i < number_of_motors_; ++i) {
    if (init_angles_degree[i] < min_angle_degree[i] || init_angles_degree[i] > max_angle_degree[i]) {
      RCLCPP_FATAL(
        get_logger(), "Initial angle %ld for motor %d exceeds limits [%ld, %ld].",
        init_angles_degree[i], i, min_angle_degree[i], max_angle_degree[i]);
      return hardware_interface::CallbackReturn::ERROR;
    } else {
      auto angle_rad = init_angles_degree[i] * (M_PI / 180.0); // Convert degrees to radians
      actuator_position_commands_rad_[i] = angle_rad; // Store in radians
      actuator_position_feedback_rad_[i] = angle_rad; // Initialize positions with initial angles
    }
  }
  actuator_velocities_rad_per_sec_ = std::vector<double>(number_of_motors_, 0.);
  actuator_efforts_ = std::vector<double>(number_of_motors_, 0.);

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

int RtRobotHardwareInterface::get_pwm_from_angle(int motor_index, double angle_rad) {
  auto delta_angle_degree = motor_params_list_[motor_index].max_angle_degree - motor_params_list_[motor_index].min_angle_degree;
  auto delta_pulse_micro = motor_params_list_[motor_index].max_pulse_micro - motor_params_list_[motor_index].min_pulse_micro;
  auto angle_degree = angle_rad * (180.0 / M_PI); // Convert radians to degrees
  // Map angle to pulse width
  int pulse_width = static_cast<int>(
      motor_params_list_[motor_index].min_pulse_micro +
      (angle_degree - motor_params_list_[motor_index].min_angle_degree) * delta_pulse_micro / delta_angle_degree);
  return pulse_width;
}

std::vector<hardware_interface::StateInterface> RtRobotHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &actuator_position_feedback_rad_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &actuator_velocities_rad_per_sec_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &actuator_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RtRobotHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &actuator_position_commands_rad_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RtRobotHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(get_logger(), "Activating ...please wait...");

  for(const auto & param : motor_params_list_) {
    RCLCPP_INFO_STREAM(
      get_logger(), "Motor: " << param.name
      << ", Channel: " << param.pwm_channel
      << ", Min Pulse: " << param.min_pulse_micro
      << " microseconds, Max Pulse: " << param.max_pulse_micro
      << " microseconds, Min Angle: " << param.min_angle_degree
      << " degrees, Max Angle: " << param.max_angle_degree
      << " degrees");
  }

  rtrobot_api_ = std::make_shared<RtRobotSerial>(
    serial_port_name_,
    baud_rate_,
    trajectory_param_);

  if (!rtrobot_api_->connect()) {
        RCLCPP_ERROR(get_logger(), "Failed to initialize RtRobotSerial. Check your serial port and baudrate.");
        rclcpp::shutdown();
        return hardware_interface::CallbackReturn::ERROR;
    }

  RCLCPP_INFO(get_logger(), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn RtRobotHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // shutdown the RtRobotSerial API
  if (rtrobot_api_) {
    rtrobot_api_->disconnect();
    RCLCPP_INFO(get_logger(), "RtRobotSerial API closed successfully.");
  } else {
    RCLCPP_WARN(get_logger(), "RtRobotSerial API was not initialized, nothing to close.");
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RtRobotHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  static std::vector<double> previous_actuator_position_rad_;
  // differentiate velocities
  if (previous_actuator_position_rad_.empty()) {
    previous_actuator_position_rad_ = actuator_position_feedback_rad_;
  } else {
    for (int index = 0; index < number_of_motors_; ++index) {
      actuator_velocities_rad_per_sec_[index] = (actuator_position_feedback_rad_[index] - previous_actuator_position_rad_[index]) / period.seconds();
      previous_actuator_position_rad_[index] = actuator_position_feedback_rad_[index];
    }
  }
  // efforts stays zero
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RtRobotHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Filter position commands to limit the rate of change
  for (int index = 0; index < number_of_motors_; ++index) {
      double command = actuator_position_commands_rad_[index];
      double command_pwm = get_pwm_from_angle(index, command);
      actuator_pwm_command_micro_sec_[index] = static_cast<int>(command_pwm);
  }
  int period_ms = static_cast<int>(std::round(period.nanoseconds() / 1e6));
  rtrobot_api_->writeData(actuator_pwm_command_micro_sec_, period_ms);

  return hardware_interface::return_type::OK;
}

}  // namespace pca9685_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  rtrobot_ros2_control::RtRobotHardwareInterface, 
  hardware_interface::SystemInterface)
