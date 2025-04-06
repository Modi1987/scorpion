#include "penta_teleop/joystick_extra_controls.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create a shared node instance
  auto joystick_extra_controls = penta_pod::teleop::joystick_extra_controls::JoystickExtraControls();

  // Multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(joystick_extra_controls.get_node());

  // Spin the executor
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
