#ifndef JOYSTICK_BASE_LINK_MOTION_HPP_
#define JOYSTICK_BASE_LINK_MOTION_HPP_
#include "base_twerk_msgs/srv/base_pose_setpoint.hpp"
#include "base_twerk_msgs/srv/get_current_base_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <rclcpp/executors.hpp>

namespace penta_pod::teleop::joystick_base_link_motion {
using JoyMsg = sensor_msgs::msg::Joy;
using PoseStampedMsg = geometry_msgs::msg::PoseStamped;
using GetCurrentBasePose = base_twerk_msgs::srv::GetCurrentBasePose;
using SetTargetBasePose = base_twerk_msgs::srv::BasePoseSetpoint;

class JoystickBaseLinkMotion {

private:
  rclcpp::Node::SharedPtr node_, clients_node_;
  rclcpp::Subscription<JoyMsg>::SharedPtr joy_subscriber_;
  rclcpp::Client<GetCurrentBasePose>::SharedPtr get_base_pose_;
  rclcpp::Client<SetTargetBasePose>::SharedPtr set_base_pose_;
  rclcpp::Publisher<PoseStampedMsg>::SharedPtr set_base_pose_pub_;

  std::shared_ptr<PoseStampedMsg> initial_base_pose;
  std::shared_ptr<PoseStampedMsg> filtered_base_pose;
  std::atomic<bool> query_pose_ready{false};
  int previous_enable_button_value{0};

  struct BaseLinkMotionParams {
    int enable_button_index{4};
    int yaw_axis_index{3};   // Right stick left/right
    int pitch_axis_index{4}; // Right stick up/down
    int x_axis_index{1};     // Left stick up/down
    int y_axis_index{0};     // Left stick left/right
    double yaw_scale{0.0};
    double pitch_scale{0.0};
    double x_scale{0.0};
    double y_scale{0.0};
    double minimum_init_z_value{0.0};
    bool command_by_topic{true};
    std::string get_base_pose_service_name;
    std::string set_base_pose_service_name;
    std::string set_base_pose_topic_name;
    double filter_value{0.1};
  } base_link_motion_params_;

  void joy_msg_to_base_link_motion(const JoyMsg::SharedPtr msg);

  void store_current_base_pose();
  void set_target_base_pose(const PoseStampedMsg::SharedPtr target_base_pose);
  void restore_initial_base_pose();

  void declare_parameters();
  void get_parameters();

public:
  JoystickBaseLinkMotion(rclcpp::Node::SharedPtr node,
                         rclcpp::Node::SharedPtr clients_node);
  void joystick_msg_callback(const JoyMsg::SharedPtr msg);
};

} // namespace penta_pod::teleop::joystick_base_link_motion
#endif // JOYSTICK_BASE_LINK_MOTION_HPP_