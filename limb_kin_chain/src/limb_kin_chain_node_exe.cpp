#include "commons/ros2_utils.hpp"
#include "limb_kin_chain/limb_kin_chain.hpp"
#include "limb_kin_chain/limb_kin_chain_node.hpp"
#include "limb_kin_chain/simple_3r_link_ik.hpp"
#include <iostream>

// Check if analytic solver arg
bool is_analytic(int argc, char **argv) {
  std::string SOLVER_KEY("--ik-solver");
  std::string SOLVER_TYPE_ANALYTIC("analytic");

  for (int i = 0; i < argc - 1; i++) {
    std::string arg(argv[i]);
    if (arg.find(SOLVER_KEY) != std::string::npos) {
      std::string solver_type_str(argv[i + 1]);
      if (solver_type_str.find(SOLVER_TYPE_ANALYTIC) != std::string::npos) {
        return true;
      }
    }
  }
  return false;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node;
  auto node_name = "limb_kin_node";

  using penta_pod::kin::commons::is_intra_process_arg;
  bool use_intra = is_intra_process_arg(argc, argv);
  if (use_intra) {
    auto node_options =
        rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    node = rclcpp::Node::make_shared(node_name, node_options);
  } else {
    node = rclcpp::Node::make_shared(node_name);
  }

  using penta_pod::kin::limb_kin_chain::Limb;             // IK using DLS
  using penta_pod::kin::limb_kin_chain::LimbIKInterface;  // IK interface
  using penta_pod::kin::limb_kin_chain::Simple3RLinkLimb; // Analytic

  std::shared_ptr<LimbIKInterface> limb;

  if (is_analytic(argc, argv)) {
    limb = std::make_shared<Simple3RLinkLimb>();
  } else {
    limb = std::make_shared<Limb>();
  }
  RCLCPP_INFO_STREAM(node->get_logger(),
                     "Limb IK solver specified is: " << limb->solver_type());

  auto limbNode =
      std::make_unique<penta_pod::kin::limb_kin_chain::LimbNode>(limb, node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}