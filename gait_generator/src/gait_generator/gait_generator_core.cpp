#include <rclcpp/executors.hpp>

// include librarries
#include "gait_generator/gait_generator.hpp"
#include "rclcpp/rclcpp.hpp"

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include <string>
#include <vector>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

#define pi 3.141592

namespace penta_pod::kin::gait_generator {

GaitGenerator::GaitGenerator()
    : node_{rclcpp::Node::make_shared("gait_generator_node")} {
  RCLCPP_INFO(node_->get_logger(), "Starting gait_generator_node");
  this->declare_parameters();
  this->load_parameters();
  const double delta_t_milli = 10.;

  current_phase_ = 0.;
  for (int i = 0; i < feet_num_; i++) {
    auto topic_name = "limb" + std::to_string(i) + "/xyz_msg";
    xyz_publishers_.push_back(
        node_->create_publisher<limb_msgs::msg::Pxyz>(topic_name, 10));
    final_displacement_.push_back(geometry_msgs::msg::Point());
  }
  phase_shift_vec_ = init_phase_shift(feet_num_);

  cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
        cmd_vel_sub_callback(msg);
      });

  cmd_null_pos_subscription_ =
      node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "null_space_pose", 10,
          [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
            cmd_null_pos_sub_callback(msg);
          });

  timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(delta_t_milli)),
      [this, delta_t_milli]() { timer_callback(delta_t_milli); });
  
  feedback_cmd_vel_publisher_ = node_->create_publisher<geometry_msgs::msg::Twist>(
      "feedback_cmd_vel", 10);

  create_set_gait_pattern_service();
}

void GaitGenerator::create_set_gait_pattern_service() {
  auto lambda =
      [this](const SetGaitPattern::Request::SharedPtr &request,
             const SetGaitPattern::Response::SharedPtr &response) -> bool {
    int pattern = request->pattern;
    if (is_walking_) {
      std::string error_message =
          "Can not change gait pattern while the robot is moving";
      RCLCPP_WARN(node_->get_logger(), error_message.c_str());
      response->message = error_message;
      response->success = false;
      return true;
    }
    if (!gait_patterns_.set_active_pattern(pattern)) {
      std::string error_message = "Could not set gait pattern to " +
                                  std::to_string(pattern) +
                                  ", make sure u r using a valid value";
      RCLCPP_WARN(node_->get_logger(), error_message.c_str());
      response->message = error_message;
      response->success = false;
      return true;
    }
    std::string info_message =
        "Changed successfully to gait pattern: " + std::to_string(pattern);
    RCLCPP_INFO(node_->get_logger(), info_message.c_str());
    response->message = info_message;
    response->success = true;

    auto log_message = gait_patterns_.active_pattern_to_string();
    RCLCPP_INFO(node_->get_logger(), log_message.c_str());
    return true;
  };
  service_callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
  set_gait_pattern_server_ = node_->create_service<SetGaitPattern>(
      "gait_generator/set_gait_pattern", lambda);
}

void GaitGenerator::cmd_vel_sub_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {

  auto mag =
      std::sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y);
  if (mag > gait_parameters_.max_gait_linear_speed) {
    msg->linear.x =
        msg->linear.x * gait_parameters_.max_gait_linear_speed / mag;
    msg->linear.y =
        msg->linear.y * gait_parameters_.max_gait_linear_speed / mag;
  }
  mag = std::abs(msg->angular.z);
  if (mag > gait_parameters_.max_gait_turning_speed) {
    msg->angular.z =
        msg->angular.z * gait_parameters_.max_gait_turning_speed / mag;
  }
  cmd_vel_ = *msg;
  /*
  RCLCPP_INFO(node_->get_logger(), "Received cmd_vel: linear.x=%.2f,
  linear.y=%.2f, angular=%.2f", cmd_vel_.linear.x, cmd_vel_.linear.y,
  cmd_vel_.angular.z);
  */
}

void GaitGenerator::cmd_null_pos_sub_callback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  // add to the initial displacement
  body_basefootprint_.translation.x = msg->pose.position.x;
  body_basefootprint_.translation.y = msg->pose.position.y;
  body_basefootprint_.translation.z = msg->pose.position.z;

  body_basefootprint_.rotation.x = msg->pose.orientation.x;
  body_basefootprint_.rotation.y = msg->pose.orientation.y;
  body_basefootprint_.rotation.z = msg->pose.orientation.z;
  body_basefootprint_.rotation.w = msg->pose.orientation.w;

  // consider the rotation absolute (initial rotation must be the identity)
  // body_basefootprint_.rotation. = msg->rotation;
  // RCLCPP_INFO(node_->get_logger(),
  //             "Received nullspace translation from equilibrium (default): "
  //             "displacement.x=%.2f, displacement.y=%.2f,
  //             displacement.z=%.2f", body_basefootprint_.translation.x,
  //             body_basefootprint_.translation.y,
  //             body_basefootprint_.translation.z);
}

void GaitGenerator::update_phase(double delta_t_milli) {

  auto delta_t_sec = delta_t_milli / 1000.;
  double w = gait_parameters_.gait_radial_frequency;

  // check if cmd_vel is zero and feet near the equilibrium
  double vel_mag = std::sqrt(cmd_vel_.linear.x * cmd_vel_.linear.x +
                             cmd_vel_.linear.y * cmd_vel_.linear.y);
  auto collective_xy_distance_from_equilibrium = 0.0;
  for (int i = 0; i < feet_num_; i++) {
    collective_xy_distance_from_equilibrium +=
        std::abs(feet_pos_in_footprint_[i].x -
                 init_feet_pos_in_footprint_[i].x) +
        std::abs(feet_pos_in_footprint_[i].y -
                 init_feet_pos_in_footprint_[i].y);
  }

  if ((collective_xy_distance_from_equilibrium < 0.005) && (vel_mag < 0.001)) {
    auto check_z_near_zero =
        current_phase_ - std::floor(current_phase_ / (2 * pi)) * 2 * pi;
    if (check_z_near_zero < w * delta_t_sec + 0.001) {
      w = 0.0;
      current_phase_ = std::floor(current_phase_ / (2 * pi)) * 2 * pi;
    }
  }
  is_walking_ = (w == 0.0) ? false : true;
  current_phase_ = current_phase_ + w * delta_t_sec;
}

void GaitGenerator::update_feet_positions(double delta_t_milli) {

  auto delta_t_sec = delta_t_milli / 1000.;

  double dx = cmd_vel_.linear.x * delta_t_sec;
  double dy = cmd_vel_.linear.y * delta_t_sec;
  double d_theta = cmd_vel_.angular.z * delta_t_sec;
  double b = gait_parameters_.step_height;
  auto gait_pattern = gait_patterns_.get_active_pattern();
  for (int i = 0; i < feet_num_; i++) {
    auto foot_index = gait_pattern[i];
    auto temp =
        foot_pos_z_generator(b, current_phase_, phase_shift_vec_[i], feet_num_);
    // Calculate feet displacement to try keep balance
    double v_mag = std::sqrt(cmd_vel_.linear.x * cmd_vel_.linear.x + cmd_vel_.linear.y * cmd_vel_.linear.y);
    double balance_motion_coef = 0.0;
    node_->get_parameter("gait_parameters.balance_internal_motion_coef", balance_motion_coef);
    double r =  balance_motion_coef * v_mag;
    double dx_balance =  r * std::sin(current_phase_ + M_PI) * delta_t_sec;
    double dy_balance = - r * std::cos(current_phase_ + M_PI) * delta_t_sec;
    // Finish balance calculation
    if (temp == 0.) {
      double x = feet_pos_in_footprint_[foot_index].x;
      double y = feet_pos_in_footprint_[foot_index].y;
      feet_pos_in_footprint_[foot_index].x = x + dx - d_theta * y + dx_balance;
      feet_pos_in_footprint_[foot_index].y = y + dy + d_theta * x + dy_balance;
      feet_pos_in_footprint_[foot_index].z = 0.;
      final_displacement_[foot_index].x =
          feet_pos_in_footprint_[foot_index].x -
          init_feet_pos_in_footprint_[foot_index].x;
      final_displacement_[foot_index].y =
          feet_pos_in_footprint_[foot_index].y -
          init_feet_pos_in_footprint_[foot_index].y;
    } else {
      feet_pos_in_footprint_[foot_index].x =
          init_feet_pos_in_footprint_[foot_index].x +
          foot_pos_xy_generator(current_phase_, phase_shift_vec_[i],
                                final_displacement_[foot_index].x, feet_num_);
      feet_pos_in_footprint_[foot_index].y =
          init_feet_pos_in_footprint_[foot_index].y +
          foot_pos_xy_generator(current_phase_, phase_shift_vec_[i],
                                final_displacement_[foot_index].y, feet_num_);
      feet_pos_in_footprint_[foot_index].z = temp;
    }
    if (foot_index < static_cast<int>(legs_body_transforms_.size())) {
      auto point = applyInverseTransform(feet_pos_in_footprint_[foot_index],
                                         body_basefootprint_);
      point = applyInverseTransform(point, legs_body_transforms_[foot_index]);
      limb_msgs::msg::Pxyz xyz_msg;
      xyz_msg.x = point.x;
      xyz_msg.y = point.y;
      xyz_msg.z = point.z;

      xyz_publishers_[foot_index]->publish(xyz_msg);
    } else {
      RCLCPP_ERROR_STREAM(node_->get_logger(),
                          "No transform available for limb " << foot_index);
    }
  }
}

void GaitGenerator::publish_base_footprint_vel_feedback() {
  auto robot_vel = geometry_msgs::msg::Twist();
  robot_vel.linear.x = - cmd_vel_.linear.x;
  robot_vel.linear.y = - cmd_vel_.linear.y;
  robot_vel.angular.z = - cmd_vel_.angular.z;
  feedback_cmd_vel_publisher_->publish(robot_vel);
}

void GaitGenerator::timer_callback(double delta_t_milli) {
  update_phase(delta_t_milli);
  update_feet_positions(delta_t_milli);
  publish_base_footprint_vel_feedback();
}

} // namespace penta_pod::kin::gait_generator
