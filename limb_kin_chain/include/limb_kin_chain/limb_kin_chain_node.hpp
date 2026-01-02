#ifndef LIMB_NODE_HPP_
#define LIMB_NODE_HPP_

#include "rclcpp/rclcpp.hpp" // for rclcpp
#include <rclcpp/executors.hpp>

// include librarries
#include "limb_kin_chain/limb_ik_interface.hpp"
#include "limb_kin_chain/limb_kin_chain.hpp"
#include "limb_kin_chain/simple_3r_link_ik.hpp"

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace penta_pod::kin::limb_kin_chain {

class LimbNode {
private:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<LimbIKInterface> limb_;
  std::vector<std::string> joints_names;
  std::vector<double> q_state;
  std::vector<double> q_target; // target setpoints
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Subscription<limb_msgs::msg::Pxyz>::SharedPtr xyz_subscriber_;
  void declare_parameters(); // shall declarte modified dh paramters

public:
  explicit LimbNode(std::shared_ptr<LimbIKInterface> limb,
                    rclcpp::Node::SharedPtr node);
  void spin() { rclcpp::spin(node_); };
};

} // namespace penta_pod::kin::limb_kin_chain

#endif // LIMB_NODE_HPP_