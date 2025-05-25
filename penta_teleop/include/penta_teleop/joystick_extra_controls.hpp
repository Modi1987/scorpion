#ifndef JOYSTICK_EXTRA_CONTROLS_HPP_
#define JOYSTICK_EXTRA_CONTROLS_HPP_

#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// include messages
#include "base_twerk_msgs/srv/base_pose_setpoint.hpp"
#include "base_twerk_msgs/srv/get_current_base_pose.hpp"
#include "gait_generator_msgs/srv/set_gait_pattern.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "base_twerk_msgs/action/base_twerk_action.hpp"

namespace penta_pod::teleop::joystick_extra_controls {

using PoseStamped = geometry_msgs::msg::PoseStamped;
using BasePoseSetpoint = base_twerk_msgs::srv::BasePoseSetpoint;
using GetCurrentBasePose = base_twerk_msgs::srv::GetCurrentBasePose;
using SetGaitPattern = gait_generator_msgs::srv::SetGaitPattern;
using BaseTwerkAction = base_twerk_msgs::action::BaseTwerkAction;
using GoalHandleBaseTwerkAction = rclcpp_action::ClientGoalHandle<BaseTwerkAction>;

class JoystickExtraControls {
private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscriber_;

  rclcpp::Client<GetCurrentBasePose>::SharedPtr get_current_base_pose_client_;
  rclcpp::Client<BasePoseSetpoint>::SharedPtr set_base_pose_client_;
  rclcpp::Client<SetGaitPattern>::SharedPtr set_gait_pattern_client_;

  rclcpp_action::Client<BaseTwerkAction>::SharedPtr base_twerk_action_client_;

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

  void send_base_twerk_goal();

  void declare_parameters();
  bool get_parameters();

  int d_pad_up_down_axis_index_{7};
  int gait_patterns_ctl_button_index_{2};
  int num_of_gait_patterns_{3};
  double z_base_max_value_{0.01};
  double z_base_min_value_{0.15};

  bool is_null_space_motion_possible_{true};

  void up_and_down_dance(const sensor_msgs::msg::Joy::SharedPtr msg);

  struct up_and_down_shake_params {
    int trigger_button_index{0};
    double magnitude{0.01}; // meters
    double w{4.0}; // rad/s
    int dance_time_millis{10000}; // milliseconds
  } up_and_down_shake_params_;

public:
  explicit JoystickExtraControls();
  auto get_node() { return node_; };
};

} // namespace penta_pod::teleop::joystick_extra_controls

#endif // JOYSTICK_EXTRA_CONTROLS_HPP_