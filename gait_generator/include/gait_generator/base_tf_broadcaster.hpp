#ifndef BASE_TF_BROADCASTER_HPP_
#define BASE_TF_BROADCASTER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>

// include messages
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace penta_pod::kin {

using PoseStamped = geometry_msgs::msg::PoseStamped;
const auto base_footprint = "base_footprint";

class BaseTfBroadcaster {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<PoseStamped>::SharedPtr base_pose_subscriber_;
  auto base_pose_sub_callback(const PoseStamped::SharedPtr msg) -> void;

public:
  explicit BaseTfBroadcaster();
  void spin() { rclcpp::spin(node_); };
};

} // namespace penta_pod::kin

#endif // BASE_TF_BROADCASTER_HPP_