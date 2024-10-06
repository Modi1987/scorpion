#ifndef JOINTS_AGGREGATOR_HPP_
#define JOINTS_AGGREGATOR_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp
#include <vector>

// include messages
#include "sensor_msgs/msg/joint_state.hpp"


namespace penta_pod::kin::joints_aggregator {
  /*
  * reveives joints angles for each limb on different topic
  * aggregates them and publishes on /joint_states
  */
  class JointsAggregator {
    private:
      rclcpp::Node::SharedPtr node_;
      std::vector<std::string> joints_states_names;
      std::vector<double> q;
      rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
      std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr> limb_joints_subscriber_;
      void declare_parameters();
      void on_joint_state_callback_limb(int limb_index, const sensor_msgs::msg::JointState& joint_state);

    public:
      explicit JointsAggregator();
      void spin() {rclcpp::spin(node_);};
  };
  
} // namespace penta_pod::kin::joints_aggregator

#endif  // JOINTS_AGGREGATOR_HPP_