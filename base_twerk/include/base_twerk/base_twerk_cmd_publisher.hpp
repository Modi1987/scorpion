#ifndef BASE_TWERK_PUBLISHER_HPP_
#define BASE_TWERK_PUBLISHER_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>

// include messages
#include "base_twerk_msgs/srv/base_pose_setpoint.hpp"
#include "base_twerk_msgs/srv/get_current_base_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace penta_pod::kin::base_twerk {

using PoseStamped = geometry_msgs::msg::PoseStamped;
using BasePoseSetpoint = base_twerk_msgs::srv::BasePoseSetpoint;
using GetCurrentBasePose = base_twerk_msgs::srv::GetCurrentBasePose;

const auto base_footprint = "base_footprint";

class BaseTwerkCmdPuplisher {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<PoseStamped>::SharedPtr base_pose_publisher_;
  rclcpp::Service<BasePoseSetpoint>::SharedPtr setpoint_service_;
  rclcpp::Service<GetCurrentBasePose>::SharedPtr get_current_base_pose_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::TimerBase::SharedPtr timer_;

  PoseStamped base_pose_;
  PoseStamped setpoint_base_pose_;
  int update_interval_millis_;
  double tracking_linear_velocity_;
  double tracking_angular_velocity_;

  void declare_parameters();
  void load_parameters();
  void timer_callback();
  void create_setpoint_service();
  void create_get_current_base_pose_service();

public:
  explicit BaseTwerkCmdPuplisher();
  void spin() { rclcpp::spin(node_); };
};

} // namespace penta_pod::kin::base_twerk

#endif // BASE_TWERK_PUBLISHER_HPP_