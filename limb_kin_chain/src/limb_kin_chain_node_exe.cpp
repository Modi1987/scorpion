#include "commons/ros2_utils.hpp"
#include "limb_kin_chain/limb_kin_chain.hpp"
#include "limb_kin_chain/limb_kin_chain_node.hpp"
#include "limb_kin_chain/simple_3r_link_ik.hpp"
#include <iostream>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;

  using penta_pod::kin::commons::is_intra_process_arg;
  bool use_intra = is_intra_process_arg(argc, argv);
  if (use_intra) {
    auto node_options =
        rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    node = rclcpp::Node::make_shared("limb_kin_node", node_options);
  } else {
    node = rclcpp::Node::make_shared("limb_kin_node");
  }

  using penta_pod::kin::limb_kin_chain::Limb;             // IK using DLS
  using penta_pod::kin::limb_kin_chain::LimbIKInterface;  // IK interface
  using penta_pod::kin::limb_kin_chain::Simple3RLinkLimb; // Analytic

  // std::shared_ptr<LimbIKInterface> limb = std::make_shared<Limb>();
  std::shared_ptr<LimbIKInterface> limb = std::make_shared<Simple3RLinkLimb>();
  auto limbNode =
      std::make_unique<penta_pod::kin::limb_kin_chain::LimbNode>(limb, node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}