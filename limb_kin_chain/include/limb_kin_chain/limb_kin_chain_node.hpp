#ifndef LIMB_NODE_HPP_
#define LIMB_NODE_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include librarries
#include "limb_kin_chain/limb_kin_chain.hpp" 

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace penta_pod::kin::limb_kin_chain {
  
  class LimbNode {
    private:
      rclcpp::Node::SharedPtr node_;
      std::shared_ptr<Limb> limb_;
      std::vector<std::string> joints_names;
      std::vector<double> q_state;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
      rclcpp::Subscription<limb_msgs::msg::Pxyz>::SharedPtr xyz_subscriber_;
      void declare_parameters(); // shall declarte modified dh paramters

    public:
      explicit LimbNode(Limb limb);
      void spin() {rclcpp::spin(node_);};
  };

} // namespace penta_pod::kin::limb_kin_chain

#endif  // LIMB_NODE_HPP_