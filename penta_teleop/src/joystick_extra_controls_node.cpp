#include "commons/ros2_utils.hpp" // here for check_intra_process_arg
#include "penta_teleop/joystick_extra_controls.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  using penta_pod::kin::commons::is_intra_process_arg;
  bool use_intra = is_intra_process_arg(argc, argv);

  // Create a shared node instance
  auto node_name = "joystick_extra_controls";
  rclcpp::Node::SharedPtr node;
  if (use_intra) {
    auto node_options =
        rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    node = rclcpp::Node::make_shared(node_name, node_options);
  } else {
    node = rclcpp::Node::make_shared(node_name);
  }

  auto joystick_extra_controls = penta_pod::teleop::joystick_extra_controls::JoystickExtraControls(node);

  // Multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  // Spin the executor
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
