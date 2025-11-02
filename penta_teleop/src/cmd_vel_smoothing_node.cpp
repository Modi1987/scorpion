#include "commons/ros2_utils.hpp" // here for check_intra_process_arg
#include "penta_teleop/cmd_vel_smoothing.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  using penta_pod::kin::commons::is_intra_process_arg;
  bool use_intra = is_intra_process_arg(argc, argv);

  auto node_name = "twist_smoothing_node";

  // Create a shared node instance
  rclcpp::Node::SharedPtr node;
  if (use_intra) {
    auto node_options =
        rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    node = rclcpp::Node::make_shared(node_name, node_options);
  } else {
    node = rclcpp::Node::make_shared(node_name);
  }

  using penta_pod::teleop::twist_smoothing::CmdVelSmoothing;
  auto twist_smoothing = CmdVelSmoothing(node);

  // Spin
  rclcpp::spin(node);

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
