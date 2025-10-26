#ifndef JOYSTICK_TURN_HEAD_USING_BUMPERS_
#define JOYSTICK_TURN_HEAD_USING_BUMPERS_
#include "base_twerk_msgs/srv/base_pose_setpoint.hpp"
#include "base_twerk_msgs/srv/get_current_base_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <rclcpp/executors.hpp>

namespace penta_pod::teleop::joystick_turn_head_using_bumpers {
using JoyMsg = sensor_msgs::msg::Joy;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using GetCurrentBasePose = base_twerk_msgs::srv::GetCurrentBasePose;
using SetTargetBasePose = base_twerk_msgs::srv::BasePoseSetpoint;

class JoystickTurnHead {

private:
  rclcpp::Node::SharedPtr node_, clients_node_;
  rclcpp::Subscription<JoyMsg>::SharedPtr joy_subscriber_;
  rclcpp::Client<GetCurrentBasePose>::SharedPtr get_base_pose_;
  rclcpp::Client<SetTargetBasePose>::SharedPtr set_base_pose_;

  std::shared_ptr<PoseStampedMsg> initial_base_pose;
  std::shared_ptr<PoseStampedMsg> filtered_base_pose;
  std::atomic<bool> query_pose_ready{false};
  int previous_enable_button_value{0};

  struct BaseLinkMotionParams {
    int enable_button_index{4};
    int yaw_turn_right_axis_index{3};   // Turn right
    int yaw_turn_left_axis_index{4};    // Turn left
    double yaw_angle_scale{0.0};
    std::string get_base_pose_service_name;
    std::string set_base_pose_service_name;
    double filter{0.1};
  } turn_left_right_motion_params_;

  double filtered_yaw_cmd{0.0};
  double previous_yaw_command{0.0};

  void joy_msg_to_base_link_motion(const JoyMsg::SharedPtr msg);

  void store_current_base_pose();
  void set_target_base_pose(const PoseStampedMsg::SharedPtr target_base_pose);
  void restore_initial_base_pose();

  void declare_parameters();
  void get_parameters();

public:
  JoystickTurnHead(rclcpp::Node::SharedPtr node,
                         rclcpp::Node::SharedPtr clients_node);
  void joystick_msg_callback(const JoyMsg::SharedPtr msg);
};

} // namespace penta_pod::teleop::joystick_turn_head_using_bumpers
#endif // JOYSTICK_TURN_HEAD_USING_BUMPERS_