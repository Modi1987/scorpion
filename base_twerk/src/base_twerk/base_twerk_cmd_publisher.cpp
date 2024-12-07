#include <rclcpp/executors.hpp>

// include librarries
#include "base_twerk/base_twerk_cmd_publisher.hpp"
#include "commons/transform_utils.hpp"
#include "rclcpp/rclcpp.hpp"

namespace penta_pod::kin::base_twerk {

BaseTwerkCmdPuplisher::BaseTwerkCmdPuplisher()
    : node_{rclcpp::Node::make_shared("base_twerk_cmd_publisher")} {
  RCLCPP_INFO(node_->get_logger(), "Starting base_twerk_cmd_publisher");
  this->declare_parameters();
  this->load_parameters();

  base_pose_publisher_ =
      node_->create_publisher<PoseStamped>("null_space_pose", 10);

  timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(update_interval_millis_)),
      [this]() { timer_callback(); });

  create_setpoint_service();
}

void BaseTwerkCmdPuplisher::create_setpoint_service() {
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  auto lambda =
      [this](const BasePoseSetpoint::Request::SharedPtr &request,
             const BasePoseSetpoint::Response::SharedPtr &response) -> bool {
    auto frame_id = request->pose.header.frame_id;
    const auto base_footprint = "base_footprint";
    if (frame_id == "") {
      frame_id = base_footprint;
    }
    if (frame_id != base_footprint) {
      response->success = false;
      response->message = frame_id + " is not a valid frame id";
      return true;
    }
    setpoint_base_pose_ = request->pose;
    RCLCPP_INFO(node_->get_logger(),
                "Base pose setpoint received x: %f | y: %f | z: %f",
                setpoint_base_pose_.pose.position.x,
                setpoint_base_pose_.pose.position.y,
                setpoint_base_pose_.pose.position.z);
    response->success = true;
    return true;
  };

  setpoint_service_ = node_->create_service<BasePoseSetpoint>(
      "cmd_null_setpoint", lambda, rmw_qos_profile_services_default,
      callback_group_);
}

void BaseTwerkCmdPuplisher::timer_callback() {
  double deta_t_sec = update_interval_millis_ / 1000.0;
  using namespace penta_pod::kin::commons;
  auto pose_optional = interpolate_pose(node_, setpoint_base_pose_, base_pose_,
                                        tracking_linear_velocity_,
                                        tracking_angular_velocity_, deta_t_sec);
  if (!pose_optional.has_value()) {
    RCLCPP_ERROR(node_->get_logger(), "Error, transofrm interpolation failed");
  } else {
    base_pose_ = pose_optional.value();
  }
  base_pose_publisher_->publish(base_pose_);
}

void BaseTwerkCmdPuplisher::declare_parameters() {
  node_->declare_parameter<int>(
      "base_twerk.base_frame_transform_update_interval_millis");
  node_->declare_parameter<double>("base_twerk.tracking_linear_velocity");
  node_->declare_parameter<double>("base_twerk.tracking_angular_velocity");
  node_->declare_parameter<std::vector<double>>(
      "init_body_basefootprint_transform", std::vector<double>{});
}

void BaseTwerkCmdPuplisher::load_parameters() {
  // Helper function to load a parameter and throw error if not found
  auto load_param = [this](const std::string &param_name, auto &param_value,
                           const std::string &error_message) {
    if (!node_->get_parameter(param_name, param_value)) {
      RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
      throw std::runtime_error(error_message);
    }
  };

  load_param(
      "base_twerk.base_frame_transform_update_interval_millis",
      update_interval_millis_,
      "No parameter base_twerk.base_frame_transform_update_interval_millis is "
      "found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.update_interval_millis is %d [milliseconds]",
              update_interval_millis_);

  load_param("base_twerk.tracking_linear_velocity", tracking_linear_velocity_,
             "No parameter base_twerk.tracking_linear_velocity is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.tracking_linear_velocity is %f [m/sec]",
              tracking_linear_velocity_);

  load_param("base_twerk.tracking_angular_velocity", tracking_angular_velocity_,
             "No parameter base_twerk.tracking_angular_velocity is found.");
  RCLCPP_INFO(node_->get_logger(),
              " base_twerk.tracking_angular_velocity is %f [rad/sec]",
              tracking_angular_velocity_);

  // Helper function to validate vector size
  auto validate_vector_size = [this](const std::vector<double> &vec,
                                     int expected_size,
                                     const std::string &error_message) {
    if (static_cast<int>(vec.size()) != expected_size) {
      RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
      throw std::runtime_error(error_message);
    }
  };

  // Load and validate initial body-to-basefootprint transform
  std::vector<double> body_basefootprint_params;
  load_param("init_body_basefootprint_transform", body_basefootprint_params,
             "No init transform body to basefootprint found.");
  validate_vector_size(body_basefootprint_params, 7,
                       "Size of init_body_basefootprint_transform must be 7 (3 "
                       "for position followed by 4 quaternion).");

  PoseStamped pose{};
  pose.pose.position.x = body_basefootprint_params[0];
  pose.pose.position.y = body_basefootprint_params[1];
  pose.pose.position.z = body_basefootprint_params[2];
  pose.pose.orientation.x = body_basefootprint_params[3];
  pose.pose.orientation.y = body_basefootprint_params[4];
  pose.pose.orientation.z = body_basefootprint_params[5];
  pose.pose.orientation.w = body_basefootprint_params[6];
  pose.header.stamp = node_->now();
  pose.header.frame_id = "base_footprint";

  base_pose_ = pose;
  setpoint_base_pose_ = pose;
}

} // namespace penta_pod::kin::base_twerk
