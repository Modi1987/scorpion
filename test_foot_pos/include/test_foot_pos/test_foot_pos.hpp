#ifndef TEST_FOOT_POS_NODE_HPP_
#define TEST_FOOT_POS_NODE_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include messages
#include "limb_msgs/msg/pxyz.hpp"

namespace penta_pod::kin::test_foot_pos {
  
  class TestFootPos {
    private:
      rclcpp::Node::SharedPtr node_;
      // std::shared_ptr<Limb> limb_;
      std::vector<double> q_state;
      double t_;
      int dof_;
      rclcpp::Publisher<limb_msgs::msg::Pxyz>::SharedPtr xyz_publisher_;
      rclcpp::TimerBase::SharedPtr timer_;
      //void declare_parameters(); // shall declarte modified dh paramters
      //void load_paramters();

    public:
      explicit TestFootPos();
      void spin() {rclcpp::spin(node_);};
  };

} // namespace penta_pod::kin::test_foot_pos

#endif  // TEST_FOOT_POS_NODE_HPP_