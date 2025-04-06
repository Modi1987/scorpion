#ifndef JOYSTICK_EXTRA_CONTROLS_HPP_
#define JOYSTICK_EXTRA_CONTROLS_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>

// include messages
#include "base_twerk_msgs/srv/base_pose_setpoint.hpp"
#include "base_twerk_msgs/srv/get_current_base_pose.hpp"
#include "gait_generator_msgs/srv/set_gait_pattern.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace penta_pod::teleop::joystick_extra_controls {

using PoseStamped = geometry_msgs::msg::PoseStamped;
using BasePoseSetpoint = base_twerk_msgs::srv::BasePoseSetpoint;
using GetCurrentBasePose = base_twerk_msgs::srv::GetCurrentBasePose;
using SetGaitPattern = gait_generator_msgs::srv::SetGaitPattern;

class JoystickExtraControls {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  rclcpp::Client<GetCurrentBasePose>::SharedPtr get_current_base_pose_client_;
  rclcpp::Client<BasePoseSetpoint>::SharedPtr set_base_pose_client_;
  rclcpp::Client<SetGaitPattern>::SharedPtr set_gait_pattern_client_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  rclcpp::TimerBase::SharedPtr timer_;

  void joy_sub_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void dpad_up_down(const sensor_msgs::msg::Joy::SharedPtr msg);
  void set_gait_pattern(const sensor_msgs::msg::Joy::SharedPtr msg);

  void handle_get_base_pose_response(
      rclcpp::Client<GetCurrentBasePose>::SharedFuture response,
      double z_disp_base_command);
  void handle_set_base_pose_response(
      rclcpp::Client<BasePoseSetpoint>::SharedFuture response);

  void declare_parameters();
  bool get_parameters();

  int d_pad_up_down_axis_index_{7};
  int gait_patterns_ctl_button_index_{2};
  int num_of_gait_patterns_{3};
  double z_base_max_value_{0.01};
  double z_base_min_value_{0.15};

public:
  explicit JoystickExtraControls();
  auto get_node() { return node_; };
};

} // namespace penta_pod::teleop::joystick_extra_controls

#endif // JOYSTICK_EXTRA_CONTROLS_HPP_