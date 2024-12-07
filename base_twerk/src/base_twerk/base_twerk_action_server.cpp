#include "base_twerk/base_twerk_action_server.hpp"

namespace penta_pod::kin::base_twerk {

using namespace std::literals::chrono_literals;

BaseTwerkActionServer::BaseTwerkActionServer()
    : node_{rclcpp::Node::make_shared("base_twerk_action_server")} {
  RCLCPP_DEBUG(node_->get_logger(), "Starting Base Twerk Action Server Node");
  this->declare_parameters();
  this->get_parameters();

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  this->setpoint_client_ = node_->create_client<BasePoseSetpointSrv>(
      "cmd_null_setpoint", rmw_qos_profile_services_default, callback_group_);

  this->base_pose_subscriber_ = node_->create_subscription<PoseStamped>(
      "null_space_pose", 10, [this](const PoseStamped &msg) -> void {
        {
          std::lock_guard<std::mutex> lock(received_base_pose_mutex_);
          received_base_pose_.timestamp = node_->now();
          received_base_pose_.value = msg;
        }
      });

  this->base_twerk_action_server_ =
      rclcpp_action::create_server<BaseTwerkAction>(
          node_, "base_twerk_action",
          [this](auto uuid, auto goal) {
            return this->handle_goal(uuid, goal);
          },
          [this](auto goal_handle) { return this->handle_cancel(goal_handle); },
          [this](auto goal_handle) {
            return this->handle_accepted(goal_handle);
          });
}

auto BaseTwerkActionServer::handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const BaseTwerkAction::Goal>)
    -> rclcpp_action::GoalResponse {
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "Received a request to twerk the base");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

auto BaseTwerkActionServer::handle_cancel(const std::shared_ptr<GoalHandle>)
    -> rclcpp_action::CancelResponse {
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

auto BaseTwerkActionServer::handle_accepted(
    const std::shared_ptr<GoalHandle> goal_handle) -> void {
  std::thread{[this, goal_handle]() {
    return this->execute(goal_handle);
  }}.detach();
}

auto BaseTwerkActionServer::execute(
    const std::shared_ptr<GoalHandle> goal_handle) -> void {
  RCLCPP_INFO(node_->get_logger(), "Executing base twerk");
  const auto goal = goal_handle->get_goal();
  auto result = std::make_shared<BaseTwerkAction::Result>();

  double mag_displacement = std::sqrt(
      goal->rx * goal->rx + goal->ry * goal->ry + goal->rz * goal->rz);
  if (mag_displacement > max_permissible_displacement_meter_) {
    auto message = "Action aborted for the specified displacement is: " +
                   std::to_string(mag_displacement) +
                   " [meter], which is bigger than the permissible value "
                   "specified in yaml as " +
                   std::to_string(max_permissible_displacement_meter_);
    result->result_message = message;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), message.c_str());
    return;
  }

  const auto millis_in_a_second = 1000.0;
  rclcpp::Rate rate(millis_in_a_second / update_interval_millis_double_);

  auto is_timed_out =
      [this](rclcpp::Time start_time,
             std::chrono::milliseconds timeout_duration_millis) -> bool {
    auto elapsed_time = node_->now() - start_time;
    auto timeout_duration_sec =
        std::chrono::duration<double>(
            std::chrono::milliseconds(timeout_duration_millis))
            .count();
    if (elapsed_time.seconds() > timeout_duration_sec) {
      return true;
    }
    return false;
  };

  // get current null pose
  auto start_time = node_->now();
  while (rclcpp::ok()) {
    rate.sleep();

    if (received_base_pose_.timestamp > start_time) {
      break;
    }

    if (is_timed_out(start_time, 1s)) {
      result->result_message = "Timed out, did not receive transform message";
      goal_handle->abort(result);
      RCLCPP_WARN(node_->get_logger(), "Time out, breaking out from action");
      return;
    }
  }

  PoseStamped start_pose{};
  {
    std::lock_guard<std::mutex> lock(received_base_pose_mutex_);
    start_pose = received_base_pose_.value;
    RCLCPP_INFO(node_->get_logger(), "Current base pose is: x:%f y:%f z:%f",
                start_pose.pose.position.x, start_pose.pose.position.y,
                start_pose.pose.position.z);
  }

  auto dance_time_millis =
      std::chrono::milliseconds(static_cast<int>(goal->dance_time_millis));

  // start control loop for base pose motion
  start_time = node_->now();
  while (rclcpp::ok()) {
    rate.sleep();

    auto current_pose =
        calculate_twerk_pose_from_goal(start_time, start_pose, goal_handle);
    if (!call_setpoint_client(current_pose)) {
      return;
    }

    if (is_timed_out(start_time, dance_time_millis)) {
      if (!call_setpoint_client(start_pose)) {
        return;
      }
      std::string msg = "Base twerk finished successfully";
      result->result_message = msg;
      goal_handle->succeed(result);
      RCLCPP_INFO_STREAM(node_->get_logger(), msg);
      return;
    }

    if (goal_handle->is_canceling()) {
      result->result_message = "action canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Goal canceled");
      return;
    }
  }
}

auto BaseTwerkActionServer::calculate_twerk_pose_from_goal(
    rclcpp::Time start_time, PoseStamped start_pose,
    const std::shared_ptr<GoalHandle> goal_handle) -> PoseStamped {
  PoseStamped pose = start_pose;
  const auto goal = goal_handle->get_goal();

  double delta_t_seconds = (node_->now() - start_time).nanoseconds() / 1e9;

  pose.pose.position.x =
      start_pose.pose.position.x +
      goal->rx * sin(goal->w * delta_t_seconds + goal->phi_x);
  pose.pose.position.y =
      start_pose.pose.position.y +
      goal->ry * sin(goal->w * delta_t_seconds + goal->phi_y);
  pose.pose.position.z =
      start_pose.pose.position.z +
      goal->rz * sin(goal->w * delta_t_seconds + goal->phi_z);

  return pose;
}

auto BaseTwerkActionServer::call_setpoint_client(
    const PoseStamped &base_to_basefootprint) -> bool {
  RCLCPP_DEBUG(node_->get_logger(), "Setting PoseStamped");
  auto request = std::make_shared<BasePoseSetpointSrv::Request>();
  request->pose.header.frame_id = "base_footprint";
  request->pose.header.stamp = node_->now();
  request->pose = base_to_basefootprint;

  const auto SERVICE_TIMEOUT_MILLIS = std::chrono::milliseconds(500);

  if (!service_call_template<BasePoseSetpointSrv>(
          node_, setpoint_client_, request, SERVICE_TIMEOUT_MILLIS)) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call setpoint client");
    return false;
  }

  return true;
}

auto BaseTwerkActionServer::declare_parameters() -> void {
  node_->declare_parameter<int>(
      "base_twerk.action_server_service_call_interval_millis");
  node_->declare_parameter<double>("base_twerk.max_permissible_displacement");
}

auto BaseTwerkActionServer::get_parameters() -> void {
  auto load_param = [this](const std::string &param_name, auto &param_value,
                           const std::string &error_message) {
    if (!node_->get_parameter(param_name, param_value)) {
      RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
      throw std::runtime_error(error_message);
    }
  };
  int update_interval_millis_int_;
  load_param("base_twerk.action_server_service_call_interval_millis",
             update_interval_millis_int_,
             "No parameter "
             "base_twerk.action_server_service_call_interval_millis is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.action_server_service_call_interval_millis is %d "
              "[milliseconds]",
              update_interval_millis_int_);
  update_interval_millis_double_ =
      static_cast<double>(update_interval_millis_int_);

  load_param("base_twerk.max_permissible_displacement",
             max_permissible_displacement_meter_,
             "No parameter base_twerk.max_permissible_displacement is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.max_permissible_displacement is %f [m]",
              max_permissible_displacement_meter_);
}

} // namespace penta_pod::kin::base_twerk
