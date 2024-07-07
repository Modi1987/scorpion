#ifndef TEST_GAIT_NODE_
#define TEST_GAIT_NODE_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include messages
#include "limb_msgs/msg/pxyz.hpp"

namespace penta_pod::kin::test_gait {
  
  class TestGait {
    private:
      rclcpp::Node::SharedPtr node_;
      // std::shared_ptr<Limb> limb_;
      std::vector<double> q_state;
      double t_, sign_;
      int dof_, foot_count_;
      std::vector<rclcpp::Publisher<limb_msgs::msg::Pxyz>::SharedPtr> xyz_publishers_;
      rclcpp::TimerBase::SharedPtr timer_;
      //void declare_parameters(); // shall declarte modified dh paramters
      //void load_paramters();

    public:
      explicit TestGait();
      void spin() {rclcpp::spin(node_);};
  };

} // namespace penta_pod::kin::test_gait

#endif  // TEST_GAIT_NODE_