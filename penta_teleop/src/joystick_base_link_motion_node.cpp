#include "penta_teleop/joystick_base_link_motion.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("joystick_base_link_motion_control");
  auto clients_node = std::make_shared<rclcpp::Node>("joystick_base_link_motion_control_clients");

  // Create a shared node instance
  using namespace penta_pod::teleop::joystick_base_link_motion;
  auto joystick_base_link_motion = JoystickBaseLinkMotion(node, clients_node);

  // Multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.add_node(clients_node);

  // Spin the executor
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
