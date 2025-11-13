#include "penta_teleop/cmd_vel_smoothing.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Create a shared node instance
  rclcpp::Node::SharedPtr node =
      rclcpp::Node::make_shared("twist_smoothing_node");

  using penta_pod::teleop::twist_smoothing::CmdVelSmoothing;
  auto twist_smoothing = CmdVelSmoothing(node);

  // Spin
  rclcpp::spin(node);

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
