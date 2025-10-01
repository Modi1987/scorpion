#include "commons/ros2_utils.hpp" // here for check_intra_process_arg
#include "gait_generator/gait_generator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  auto node_name = "gait_generator_node";

  using penta_pod::kin::commons::is_intra_process_arg;
  bool use_intra = is_intra_process_arg(argc, argv);
  if (use_intra) {
    auto node_options =
        rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    node = rclcpp::Node::make_shared(node_name, node_options);
  } else {
    node = rclcpp::Node::make_shared(node_name);
  }

  auto gait_generator = penta_pod::kin::gait_generator::GaitGenerator(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}