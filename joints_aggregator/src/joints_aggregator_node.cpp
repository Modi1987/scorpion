#include "commons/ros2_utils.hpp" // here for check_intra_process_arg
#include "joints_aggregator/joints_aggregator.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  auto node_name = "joints_aggregator_node";

  using penta_pod::kin::commons::is_intra_process_arg;
  bool use_intra = is_intra_process_arg(argc, argv);
  if (use_intra) {
    auto node_options =
        rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    node = rclcpp::Node::make_shared(node_name, node_options);
  } else {
    node = rclcpp::Node::make_shared(node_name);
  }
  auto joints_aggregator =
      penta_pod::kin::joints_aggregator::JointsAggregator(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}