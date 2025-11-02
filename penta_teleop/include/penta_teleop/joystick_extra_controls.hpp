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

  /* up down motion, funs/vars */
  int d_pad_up_down_axis_index_{7};
  int gait_patterns_ctl_button_index_{2};
  int num_of_gait_patterns_{3};
  double z_base_max_value_{0.01};
  double z_base_min_value_{0.15};

  void joy_sub_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void dpad_up_down(const sensor_msgs::msg::Joy::SharedPtr msg);
  void set_gait_pattern(const sensor_msgs::msg::Joy::SharedPtr msg);

  /* get/set base pose */
  void handle_get_base_pose_response(
      rclcpp::Client<GetCurrentBasePose>::SharedFuture response,
      double z_disp_base_command, double pitch_disp_base_command);
  void handle_set_base_pose_response(
      rclcpp::Client<BasePoseSetpoint>::SharedFuture response);

  /* twerk funs */
  void send_base_twerk_goal(int twerk_axis_index, double twerk_magnitude, int twerk_time_millis);
  
  bool is_null_space_motion_possible_{true};
  void check_twerk_it_mode_switch(const sensor_msgs::msg::Joy::SharedPtr msg);
  void check_twerk_pressed(const sensor_msgs::msg::Joy::SharedPtr msg);


  /* parameters funs */
  void declare_parameters();
  bool get_parameters();
  void declare_base_height_parameters();
  bool get_base_height_parameters();
  void declare_gait_patterns_parameters();
  bool get_gait_patterns_parameters();
  void declare_twerk_it_parameters();
  bool get_twerk_it_parameters();

  /* twerk it params struct */
  struct TwerkItParams {      
    int trigger_button_index{0}; // button to trigger twerk
    int switch_mode_button_index{1}; // button to switch twerk modes
    double w{4.0}; // rad/s
    int dance_time_millis{10000}; // milliseconds
    int current_twerk_mode_index{0};

    std::vector<double> magnitudes;
    std::vector<std::string> twerk_mode_names;
    std::vector<long int> twerk_mode_axes;

    /* get joystick buttons indicies*/
    int get_twerk_it_trigger_button_index() {
      return trigger_button_index;
    }

    int get_twerk_it_switch_mode_button_index() {
      return switch_mode_button_index;
    }
    
    /* get parameters */
    auto check_current_twerk_it_mode_index() -> bool {
      if (current_twerk_mode_index < 0 ||
          static_cast<size_t>(current_twerk_mode_index) >= twerk_mode_names.size()) {
        RCLCPP_ERROR(rclcpp::get_logger("JoystickExtraControls"),
                     "Current twerk mode index is out of bounds : %d resetting it to zero!", 
                     current_twerk_mode_index);
        current_twerk_mode_index = 0;
        return false;
      }
      return true;
    }

    auto get_current_twerk_it_mode_name() -> std::string {
      if(!check_current_twerk_it_mode_index()) {
        RCLCPP_ERROR(rclcpp::get_logger("fun get_current_twerk_it_mode_name"),
                     "Failed to assert current twerk mode index. Out of bounds");
      }
      return twerk_mode_names[current_twerk_mode_index];
    }

    auto get_current_twerk_it_axis() -> int {
      if(!check_current_twerk_it_mode_index()) {
        RCLCPP_ERROR(rclcpp::get_logger("fun get_current_twerk_it_axis"),
                     "Failed to assert current twerk mode index. Out of bounds");
      }
      return twerk_mode_axes[current_twerk_mode_index];
    }

    auto get_current_twerk_it_magnitude() -> double {
      if(!check_current_twerk_it_mode_index()) {
        RCLCPP_ERROR(rclcpp::get_logger("fun get_current_twerk_it_magnitude"),
                     "Failed to assert current twerk mode index. Out of bounds");
      }
      return magnitudes[current_twerk_mode_index];
    }

    int get_current_twerk_it_dance_time_millis() {
      return dance_time_millis;
    }

    /* set some stuff */
    int switch_twerk_it_mode() {
      current_twerk_mode_index++;
      if (static_cast<size_t>(current_twerk_mode_index) >= twerk_mode_names.size()) {
        current_twerk_mode_index = 0;
      }
      return current_twerk_mode_index;
    }

  } twerk_it_params_;

public:
  explicit JoystickExtraControls(rclcpp::Node::SharedPtr node);
};

} // namespace penta_pod::teleop::joystick_extra_controls

#endif // JOYSTICK_EXTRA_CONTROLS_HPP_