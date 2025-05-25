#ifndef JOINTS_AGGREGATOR_HPP_
#define JOINTS_AGGREGATOR_HPP_

#include "rclcpp/rclcpp.hpp" // for rclcpp
#include <rclcpp/executors.hpp>
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
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr>
      limb_joints_subscriber_;
  void declare_parameters();
  void
  on_joint_state_callback_limb(int limb_index,
                               const sensor_msgs::msg::JointState &joint_state);

  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex q_mutex_;
  std::vector<double> q_;
  std::vector<double> previous_q_;
  int limbs_num_;
  std::vector<long int> joints_per_limb_;
  int update_interval_millis_;
  int number_of_messages_forcely_published_on_startup_ {0};
  int publish_on_startup_counter_ {0};
  bool publish_joints_only_on_value_change_{false};

  auto get_parameters() -> bool;

public:
  explicit JointsAggregator();
  void spin() { rclcpp::spin(node_); };
};

} // namespace penta_pod::kin::joints_aggregator

#endif // JOINTS_AGGREGATOR_HPP_