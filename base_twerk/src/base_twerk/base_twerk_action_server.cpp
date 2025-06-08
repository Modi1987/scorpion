#include "base_twerk/base_twerk_action_server.hpp"
#include "base_twerk/base_twerk_helper_funs.hpp"
#include "commons/quaternion_utils.hpp"

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

  this->get_currnet_pose_client_ = node_->create_client<GetCurrentBasePose>(
      "get_current_null_pose", rmw_qos_profile_services_default,
      callback_group_);

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

  // check if target is valid
  if (!is_target_valid(goal_handle)) {
    RCLCPP_ERROR(node_->get_logger(), "Invalid target, action aborted");
    return;
  }

  const auto millis_in_a_second = 1000.0;
  rclcpp::Rate rate(millis_in_a_second /
                    twerk_yaml_configs_.update_interval_millis_double_);

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

  // get current null space pose
  auto get_currnet_pose_response = quiry_current_base_pose();
  if (!get_currnet_pose_response.has_value()) {
    result->result_message = "Failed to get current base pose";
    goal_handle->abort(result);
    return;
  }

  PoseStamped start_pose{};
  {
    std::lock_guard<std::mutex> lock(received_base_pose_mutex_);
    received_base_pose_ = {node_->now(),
                           get_currnet_pose_response.value().pose};
    start_pose = received_base_pose_.value;
    RCLCPP_INFO(node_->get_logger(), "Current base pose is: x:%f y:%f z:%f",
                start_pose.pose.position.x, start_pose.pose.position.y,
                start_pose.pose.position.z);
  }

  auto dance_time_millis =
      get_rounded_dance_time_from_goal_millis(goal->w, goal->dance_time_millis);

  // start control loop for base pose motion
  auto start_time = node_->now();
  while (rclcpp::ok()) {
    rate.sleep();

    auto updated_pose =
        calculate_twerk_pose_from_goal(start_time, start_pose, goal_handle);
    if (!call_setpoint_client(updated_pose)) {
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
      if (!call_setpoint_client(start_pose)) {
        return;
      }
      result->result_message = "action canceled";
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Goal canceled");
      return;
    }
  }
}

auto BaseTwerkActionServer::quiry_current_base_pose()
    -> std::optional<GetCurrentBasePose::Response> {

  // check if service is available
  auto service_name = get_currnet_pose_client_->get_service_name();
  if (!get_currnet_pose_client_->wait_for_service(1s)) {
    RCLCPP_ERROR(node_->get_logger(), "Service %s not online!", service_name);
    return std::nullopt;
  }

  // send request
  auto get_current_pose_request =
      std::make_shared<GetCurrentBasePose::Request>();

  auto start_time = node_->now();

  auto future =
      get_currnet_pose_client_->async_send_request(get_current_pose_request);

  using namespace std::chrono_literals;
  const auto timeout_millis = std::chrono::milliseconds(500);

  auto status = future.wait_for(timeout_millis);

  if (status != std::future_status::ready) {
    RCLCPP_ERROR(node_->get_logger(), "Service %s response timed out!",
                 service_name);
    return std::nullopt;
  }

  if (!future.valid()) {
    RCLCPP_ERROR(node_->get_logger(), "Future from service %s is not valid!",
                 service_name);
    return std::nullopt;
  }

  auto result = future.get();
  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Returned null Service %s response!",
                 service_name);
    return std::nullopt;
  }

  RCLCPP_INFO(node_->get_logger(), "Service %s response is ready!",
              service_name);
  return *result;
}

auto BaseTwerkActionServer::calculate_twerk_pose_from_goal(
    rclcpp::Time start_time, PoseStamped start_pose,
    const std::shared_ptr<GoalHandle> goal_handle) -> PoseStamped {
  PoseStamped pose = start_pose;
  const auto goal = goal_handle->get_goal();

  double delta_t_seconds = (node_->now() - start_time).nanoseconds() / 1e9;

  pose.pose.position.x =
      start_pose.pose.position.x +
      goal->r[0] * sin(goal->w * delta_t_seconds + goal->phi[0]);
  pose.pose.position.y =
      start_pose.pose.position.y +
      goal->r[1] * sin(goal->w * delta_t_seconds + goal->phi[1]);
  pose.pose.position.z =
      start_pose.pose.position.z +
      goal->r[2] * sin(goal->w * delta_t_seconds + goal->phi[2]);

  std::vector<double> rpy = {0., 0., 0.};
  for (int i = 3; i < 6; i++) {
    rpy[i - 3] = goal->r[i] * sin(goal->w * delta_t_seconds + goal->phi[i]);
  }

  auto roll = rpy[0];
  auto pitch = rpy[1];
  auto yaw = rpy[2];
  pose.pose.orientation = penta_pod::kin::commons::quaternion_utils::rpy_to_quaternion(yaw, pitch, roll);
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

auto BaseTwerkActionServer::is_target_valid(
    const std::shared_ptr<GoalHandle> goal_handle) -> bool {
  // Check goal handle validity
  if (!goal_handle) {
    RCLCPP_ERROR(node_->get_logger(), "Goal handle is null");
    return false;
  }

  auto result = std::make_shared<BaseTwerkAction::Result>();

  const auto goal = goal_handle->get_goal();

  // Check input fields
  if (goal->w == 0.0) {
    auto message = "Action target goal for the specified frequency is: " +
                   std::to_string(goal->w) +
                   " [Hz], which is not valid. It should not be 0.";
    result->result_message = message;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), message.c_str());
    return false;
  }

  if (goal->dance_time_millis <= 0) {
    auto message =
        "Action target goal for the specified dance time is: " +
        std::to_string(goal->dance_time_millis) +
        " [milliseconds], which is not valid. It should be positive.";
    result->result_message = message;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), message.c_str());
    return false;
  }

  double mag_displacement =
      std::sqrt(goal->r[0] * goal->r[0] + goal->r[1] * goal->r[1] +
                goal->r[2] * goal->r[2]);
  double comparision_value =
      twerk_yaml_configs_.max_permissible_displacement_meter_;
  if (mag_displacement > comparision_value) {
    auto message = "The specified displacement magnitude is: " +
                   std::to_string(mag_displacement) +
                   " [meter], which is bigger than the permissible value "
                   "specified in yaml as " +
                   std::to_string(comparision_value);
    result->result_message = message;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), message.c_str());
    return false;
  }

  double mag_rotation =
      std::sqrt(goal->r[3] * goal->r[3] + goal->r[4] * goal->r[4] +
                goal->r[5] * goal->r[5]);
  comparision_value = twerk_yaml_configs_.max_permissible_rotation_rad_;
  if (mag_rotation > comparision_value) {
    auto message =
        "The specified rotation magnitude is: " + std::to_string(mag_rotation) +
        " [rad], which is bigger than the permissible value "
        "specified in yaml as " +
        std::to_string(comparision_value);
    result->result_message = message;
    goal_handle->abort(result);
    RCLCPP_ERROR(node_->get_logger(), message.c_str());
    return false;
  }

  return true;
}

auto BaseTwerkActionServer::declare_parameters() -> void {
  node_->declare_parameter<int>(
      "base_twerk.action_server_service_call_interval_millis");
  node_->declare_parameter<double>("base_twerk.max_permissible_displacement");
  node_->declare_parameter<double>("base_twerk.max_permissible_rotation");
  node_->declare_parameter<double>("base_twerk.min_permissible_displacement");
  node_->declare_parameter<double>("base_twerk.min_permissible_rotation");
}

auto BaseTwerkActionServer::get_parameters() -> void {
  auto load_param = [this](const std::string &param_name, auto &param_value,
                           const std::string &error_message) {
    if (!node_->get_parameter(param_name, param_value)) {
      RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
      throw std::runtime_error(error_message);
    }
  };
  int update_interval_millis_int;
  load_param("base_twerk.action_server_service_call_interval_millis",
             update_interval_millis_int,
             "No parameter "
             "base_twerk.action_server_service_call_interval_millis is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.action_server_service_call_interval_millis is %d "
              "[milliseconds]",
              update_interval_millis_int);
  twerk_yaml_configs_.update_interval_millis_double_ =
      static_cast<double>(update_interval_millis_int);

  load_param("base_twerk.max_permissible_displacement",
             twerk_yaml_configs_.max_permissible_displacement_meter_,
             "No parameter base_twerk.max_permissible_displacement is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.max_permissible_displacement is %f [m]",
              twerk_yaml_configs_.max_permissible_displacement_meter_);

  load_param("base_twerk.max_permissible_rotation",
             twerk_yaml_configs_.max_permissible_rotation_rad_,
             "No parameter base_twerk.max_permissible_rotation is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.max_permissible_rotation is %f [rad]",
              twerk_yaml_configs_.max_permissible_rotation_rad_);

  load_param("base_twerk.min_permissible_displacement",
             twerk_yaml_configs_.min_permissible_displacement_meter_,
             "No parameter base_twerk.min_permissible_displacement is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.min_permissible_displacement is %f [m]",
              twerk_yaml_configs_.min_permissible_displacement_meter_);

  load_param("base_twerk.min_permissible_rotation",
             twerk_yaml_configs_.min_permissible_rotation_rad_,
             "No parameter base_twerk.min_permissible_rotation is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.min_permissible_rotation is %f [rad]",
              twerk_yaml_configs_.min_permissible_rotation_rad_);
}

} // namespace penta_pod::kin::base_twerk
