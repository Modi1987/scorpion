#ifndef BASE_TWERK_ACTION_SERVER_HPP_
#define BASE_TWERK_ACTION_SERVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <rclcpp/executors.hpp>

#include "commons/ros2_utils.hpp"

#include "base_twerk_msgs/action/base_twerk_action.hpp"
#include "base_twerk_msgs/srv/base_pose_setpoint.hpp"
#include "base_twerk_msgs/srv/get_current_base_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace penta_pod::kin::base_twerk {

using BaseTwerkAction = base_twerk_msgs::action::BaseTwerkAction;
using GoalHandle = rclcpp_action::ServerGoalHandle<BaseTwerkAction>;
using BasePoseSetpointSrv = base_twerk_msgs::srv::BasePoseSetpoint;
using GetCurrentBasePose = base_twerk_msgs::srv::GetCurrentBasePose;
using geometry_msgs::msg::PoseStamped;
using penta_pod::kin::commons::service_call_template;

class BaseTwerkActionServer {
private:
  rclcpp::Node::SharedPtr node_;

  rclcpp_action::Server<BaseTwerkAction>::SharedPtr base_twerk_action_server_;

  rclcpp::Client<BasePoseSetpointSrv>::SharedPtr setpoint_client_;

  rclcpp::Client<GetCurrentBasePose>::SharedPtr get_currnet_pose_client_;

  rclcpp::CallbackGroup::SharedPtr callback_group_;

  auto handle_goal(const rclcpp_action::GoalUUID &,
                   std::shared_ptr<const BaseTwerkAction::Goal>)
      -> rclcpp_action::GoalResponse;

  auto handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
      -> rclcpp_action::CancelResponse;

  auto handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) -> void;

  auto execute(const std::shared_ptr<GoalHandle> goal_handle) -> void;

  auto is_target_valid(const std::shared_ptr<GoalHandle> goal_handle) -> bool;

  auto declare_parameters() -> void;
  auto get_parameters() -> void;

  auto call_setpoint_client(const PoseStamped &base_to_basefootprint) -> bool;

  auto calculate_twerk_pose_from_goal(
      rclcpp::Time start_time, PoseStamped start_pose,
      const std::shared_ptr<GoalHandle> goal_handle) -> PoseStamped;

  auto quiry_current_base_pose() -> std::optional<GetCurrentBasePose::Response>;

  struct TwerkYamlConfigs {
    double update_interval_millis_double_;
    double max_permissible_displacement_meter_;
    double min_permissible_displacement_meter_;
    double max_permissible_rotation_rad_;
    double min_permissible_rotation_rad_;
  } twerk_yaml_configs_;

  struct ReceviedPoseStamped {
    rclcpp::Time timestamp;
    PoseStamped value;
  } received_base_pose_;
  std::mutex received_base_pose_mutex_;

public:
  explicit BaseTwerkActionServer();
  void spin() { rclcpp::spin(node_); };
};
} // namespace penta_pod::kin::base_twerk

#endif // BASE_TWERK_ACTION_SERVER_HPP_