#include "gait_generator/base_tf_broadcaster.hpp"

namespace penta_pod::kin {

BaseTfBroadcaster::BaseTfBroadcaster()
    : node_{rclcpp::Node::make_shared("base_tf_broadcaster")} {
  RCLCPP_INFO(node_->get_logger(),
              "Starting base_tf_broadcaster, subscriping to null_space_pose "
              "topic and broadcasting tf");
  base_pose_subscriber_ = node_->create_subscription<PoseStamped>(
      "null_space_pose", 10, [this](const PoseStamped::SharedPtr msg) {
        this->base_pose_sub_callback(msg);
      });
}

auto BaseTfBroadcaster::base_pose_sub_callback(const PoseStamped::SharedPtr msg)
    -> void {
  geometry_msgs::msg::TransformStamped transformStamped;
  // header
  transformStamped.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
  transformStamped.header.frame_id = name_space_ + "base_footprint";
  transformStamped.child_frame_id = name_space_ + "base_link";
  // translation
  transformStamped.transform.translation.x = msg->pose.position.x;
  transformStamped.transform.translation.y = msg->pose.position.y;
  transformStamped.transform.translation.z = msg->pose.position.z;
  // rotation
  transformStamped.transform.rotation.x = msg->pose.orientation.x;
  transformStamped.transform.rotation.y = msg->pose.orientation.y;
  transformStamped.transform.rotation.z = msg->pose.orientation.z;
  transformStamped.transform.rotation.w = msg->pose.orientation.w;
  // broadcast
  static auto br = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  br->sendTransform(transformStamped);
}

auto BaseTfBroadcaster::declare_parameters() -> void {
  node_->declare_parameter("name_space", "");
}

auto BaseTfBroadcaster::get_parameters() -> void {
  node_->get_parameter("name_space", name_space_);
  RCLCPP_INFO(node_->get_logger(),
              "BaseTfBroadcaster: name_space: %s",
              name_space_.c_str());
}

} // namespace penta_pod::kin