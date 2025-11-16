#ifndef RT_ROBOT_HW_INTERFACE_HPP_
#define RT_ROBOT_HW_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rtrobot_interfaces/msg/motor_channel_params.hpp"
#include "rtrobot_ros2_control/rtrobot_serial_api.hpp"

#include <iostream>

using MotorParams = rtrobot_interfaces::msg::MotorChannelParams;

namespace rtrobot_ros2_control
{

class RtRobotHardwareInterface : public hardware_interface::SystemInterface
{
public:

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/) override;

  /// Get the logger of the SystemInterface.
  /**
   * \return logger of the SystemInterface.
   */
  rclcpp::Logger get_logger() const { return *logger_; }

  /// Get the clock of the SystemInterface.
  /**
   * \return clock of the SystemInterface.
   */
  rclcpp::Clock::SharedPtr get_clock() const { return clock_; }

private:
  // Objects for logging
  std::shared_ptr<rclcpp::Logger> logger_;
  rclcpp::Clock::SharedPtr clock_;

  // Store command and joints feedback
  std::vector<double> actuator_position_commands_rad_; // setpoints received from ros2_control
  std::vector<double> actuator_position_feedback_rad_;
  std::vector<double> actuator_velocities_rad_per_sec_;
  std::vector<double> actuator_efforts_;
  std::vector<int> actuator_pwm_command_micro_sec_;

  // Motors parameters
  std::string serial_port_name_;
  int baud_rate_{115200};
  int number_of_motors_{0};
  std::vector<MotorParams> motor_params_list_;  
  std::shared_ptr<RtRobotSerial> rtrobot_api_;
  std::string trajectory_param_{"T10D800"};

  void flush_position_commands_to_motors();

  int get_pwm_from_angle(int motor_index, double angle_rad);

  template <typename T>
  void log_parsed_list(const std::vector<T>& list, const std::string& name) const {
    std::ostringstream oss;
    oss << name << " vector has " << list.size() << " selements : [";
    for (size_t i = 0; i < list.size(); ++i) {
      oss << list[i];
      if (i < list.size() - 1) {
        oss << ", ";
      }
    }
    oss << "]";
    std::cout << oss.str();
  }

};

}  // namespace dynamixel_harware_interface

#endif  // RT_ROBOT_HW_INTERFACE_HPP_
