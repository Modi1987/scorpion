#ifndef PCA9695_HW_INTERFACE_HPP_
#define PCA9695_HW_INTERFACE_HPP_

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

#include "pca9685_interfaces/msg/pca_channel_params.hpp"
#include "pca9685_ros2_control/pca9685_ros2_api.hpp"

#include <iostream>

using MotorParams = pca9685_interfaces::msg::PcaChannelParams;

namespace pca9685_ros2_control
{

class Pca9685HardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Pca9685HardwareInterface);

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

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
  std::vector<double> actuator_position_filtered_rad_; // filtered commands sent to motors
  std::vector<double> actuator_positions_feedback_rad_;
  std::vector<double> actuator_velocities_rad_per_sec_;
  std::vector<double> actuator_efforts_;

  // Motors parameters
  std::string i2c_device_name_;
  int i2c_address_{0x40};  // Default I2C address for PCA9685
  int number_of_motors_{0};
  std::vector<MotorParams> motor_params_list_;  
  std::shared_ptr<MyPCA9685> pca_api_;

  void flush_position_comamnds_to_motors();

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

#endif  // PCA9695_HW_INTERFACE_HPP_
