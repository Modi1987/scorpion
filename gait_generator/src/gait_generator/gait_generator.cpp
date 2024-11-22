#include <rclcpp/executors.hpp>

// include librarries 
#include "gait_generator/gait_generator.hpp"  
#include "rclcpp/rclcpp.hpp"     

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include <vector>
#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <stdexcept>

#define pi 3.141592

namespace penta_pod::kin::gait_generator {

  GaitGenerator::GaitGenerator() : node_{rclcpp::Node::make_shared("gait_generator_node")}
  {
    RCLCPP_INFO(node_->get_logger(), "Starting gait_generator_node");
    this->declare_parameters();
    this->load_parameters();
    const double delta_t_milli = 50.;

    current_phase_ = 0.;
    for(int i = 0; i < feet_num_; i++) {
        auto topic_name = "limb" + std::to_string(i) + "/xyz_msg";
        xyz_publishers_.push_back(node_->create_publisher<limb_msgs::msg::Pxyz>(topic_name, 10));
        final_displacement_.push_back(geometry_msgs::msg::Point());
    }
    phase_shift_vec_ = init_phase_shift(feet_num_);

    cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg){ cmd_vel_sub_callback(msg); });

    cmd_null_pos_subscription_ = node_->create_subscription<geometry_msgs::msg::Transform>(
        "cmd_null_position", 10, [this](const geometry_msgs::msg::Transform::SharedPtr msg){ cmd_null_pos_sub_callback(msg); });

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delta_t_milli)),
        [this, delta_t_milli]() { timer_callback(delta_t_milli); });
  }

  void GaitGenerator::cmd_vel_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    auto mag = std::sqrt(msg->linear.x * msg->linear.x + msg->linear.y * msg->linear.y);
    if (mag > max_gait_linear_speed_) {
        msg->linear.x = msg->linear.x * max_gait_linear_speed_ / mag;
        msg->linear.y = msg->linear.y * max_gait_linear_speed_ / mag;
    }
    mag = std::abs(msg->angular.z);
    if (mag > max_gait_turning_speed_) {
        msg->angular.z = msg->angular.z * max_gait_turning_speed_ / mag;
    }
    cmd_vel_ = *msg;
    /*
    RCLCPP_INFO(node_->get_logger(), "Received cmd_vel: linear.x=%.2f, linear.y=%.2f, angular=%.2f",
                cmd_vel_.linear.x, cmd_vel_.linear.y, cmd_vel_.angular.z);
    */
  }

  void GaitGenerator::cmd_null_pos_sub_callback(const geometry_msgs::msg::Transform::SharedPtr msg) {
    // add to the initial displacement
    body_basefootprint_.translation.x = init_body_basefootprint_.translation.x + msg->translation.x;
    body_basefootprint_.translation.y = init_body_basefootprint_.translation.y + msg->translation.y;
    body_basefootprint_.translation.z = init_body_basefootprint_.translation.z + msg->translation.z;
    // consider the rotation absolute (initial rotation must be the identity)
    body_basefootprint_.rotation = msg->rotation;
    RCLCPP_INFO(node_->get_logger(), "Received nullspace translation from equilibrium (default): displacement.x=%.2f, displacement.y=%.2f, displacement.z=%.2f",
                msg->translation.x, msg->translation.y, msg->translation.z);
  }

  void GaitGenerator::timer_callback(double delta_t_milli){
      auto delta_t_sec = delta_t_milli/1000.;
      double dx = cmd_vel_.linear.x*delta_t_sec;
      double dy = cmd_vel_.linear.y*delta_t_sec;
      double d_theta = cmd_vel_.angular.z*delta_t_sec;
      double w = 2.5;
      double b = 0.05;
      current_phase_=current_phase_+w*delta_t_sec;

      for(int i = 0; i < feet_num_; i++) {
          auto temp = stepFunOneLegOff_2(b,current_phase_,phase_shift_vec_[i],feet_num_);
          if(temp==0.) {
              double x = feet_pos_in_footprint_[i].x;
              double y = feet_pos_in_footprint_[i].y;
              feet_pos_in_footprint_[i].x = x + dx - d_theta*y;
              feet_pos_in_footprint_[i].y = y + dy + d_theta*x;
              feet_pos_in_footprint_[i].z = feet_pos_in_footprint_[i].z;
              final_displacement_[i].x = feet_pos_in_footprint_[i].x - init_feet_pos_in_footprint_[i].x;
              final_displacement_[i].y = feet_pos_in_footprint_[i].y - init_feet_pos_in_footprint_[i].y;
          } else {
              feet_pos_in_footprint_[i].x = init_feet_pos_in_footprint_[i].x + 
                                              stepFunOneLegOff_3(current_phase_, phase_shift_vec_[i], final_displacement_[i].x, feet_num_);
              feet_pos_in_footprint_[i].y = init_feet_pos_in_footprint_[i].y + 
                                              stepFunOneLegOff_3(current_phase_, phase_shift_vec_[i], final_displacement_[i].y, feet_num_);
              feet_pos_in_footprint_[i].z = temp;
          }

          if (i < static_cast<int>(legs_body_transforms_.size())) {
            auto  point = applyInverseTransform(feet_pos_in_footprint_[i], body_basefootprint_);
            point = applyInverseTransform(point, legs_body_transforms_[i]);
            limb_msgs::msg::Pxyz xyz_msg;
            xyz_msg.x = point.x;
            xyz_msg.y = point.y;
            xyz_msg.z = point.z;

            xyz_publishers_[i]->publish(xyz_msg);
          } else {
              RCLCPP_ERROR_STREAM(node_->get_logger(), "No transform available for limb " << i);
          }
      }    
  }
  
  void GaitGenerator::declare_parameters(){
    node_->declare_parameter<int>("limbs_num");
    node_->declare_parameter<std::vector<double>>("legs_body_transforms", std::vector<double>{});
    node_->declare_parameter<std::vector<double>>("init_feet_pos_in_basefootprint", std::vector<double>{});
    node_->declare_parameter<std::vector<double>>("init_body_basefootprint_transform", std::vector<double>{});
    node_->declare_parameter<double>("gait_parameters.max_gait_linear_speed");
    node_->declare_parameter<double>("gait_parameters.max_gait_turning_speed");
  }

  
  void GaitGenerator::load_parameters() {
      // robot kinematic parameters
      if (!node_->get_parameter("limbs_num", feet_num_))
      {
          const char* message = "No limbs_num parameter found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      RCLCPP_INFO(node_->get_logger(), "Loaded limbs_num value is: %d", feet_num_);
      std::vector<double> transform_params;
      if (!node_->get_parameter("legs_body_transforms", transform_params))
      {
          const char* message = "No transform parameters found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      if (static_cast<int>(transform_params.size()) != feet_num_ * 7)
      {
          std::string message = "Invalid transform parameter size, expected " + std::to_string(feet_num_) + "x7 elements.";
          RCLCPP_ERROR(node_->get_logger(), message.c_str());
          throw std::runtime_error(message.c_str());
      }
      for (int count = 0; count < feet_num_; count++)
      {
          geometry_msgs::msg::Transform transform;
          transform.translation.x = transform_params[0 + 7 * count];
          transform.translation.y = transform_params[1 + 7 * count];
          transform.translation.z = transform_params[2 + 7 * count];
          transform.rotation.x = transform_params[3 + 7 * count];
          transform.rotation.y = transform_params[4 + 7 * count];
          transform.rotation.z = transform_params[5 + 7 * count];
          transform.rotation.w = transform_params[6 + 7 * count];
          legs_body_transforms_.emplace_back(transform);
          // log some usefull info
          std::string message = "Shoulder [" + std::to_string(count) + "] transform in body frame is: \n";
          message = message + "[ "; 
          for (int i = 0; i < 7; i++) {
            auto val = transform_params[i + 7 * count];
            message = message +  double_to_string_formatted(val, 4) + " ";
          }
          message = message + "]";
          RCLCPP_INFO(node_->get_logger(), message.c_str());
      }

      std::vector<double> init_feet_pos_params;
      if (!node_->get_parameter("init_feet_pos_in_basefootprint", init_feet_pos_params))
      {
          const char* message = "No feet position parameters found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      if (static_cast<int>(init_feet_pos_params.size()) != feet_num_ * 3)
      {
          std::string message = "Invalid feet position parameter size, expected " + std::to_string(feet_num_) + "x3 elements.";
          RCLCPP_ERROR(node_->get_logger(), message.c_str());
          throw std::runtime_error(message.c_str());
      }
      for (int count = 0; count < 7; count++)
      {
          geometry_msgs::msg::Point xyz;
          xyz.x = init_feet_pos_params[0 + count * 3];
          xyz.y = init_feet_pos_params[1 + count * 3];
          xyz.z = init_feet_pos_params[2 + count * 3];
          feet_pos_in_footprint_.emplace_back(xyz);
          init_feet_pos_in_footprint_.emplace_back(xyz);
      }

      std::vector<double> body_basefootprint_params;
      if (!node_->get_parameter("init_body_basefootprint_transform", body_basefootprint_params)) 
      {
          const char* message = "No init transform body to basefootprint found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      if (body_basefootprint_params.size()!=7)
      {
          const char* message = "Size of init_body_basefootprint_transform must be 7 (3 for position followed by 4 quaternion).";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      } else {
          geometry_msgs::msg::Transform transform;
          transform.translation.x = body_basefootprint_params[0];
          transform.translation.y = body_basefootprint_params[1];
          transform.translation.z = body_basefootprint_params[2];
          transform.rotation.x = body_basefootprint_params[3];
          transform.rotation.y = body_basefootprint_params[4];
          transform.rotation.z = body_basefootprint_params[5];
          transform.rotation.w = body_basefootprint_params[6];
          init_body_basefootprint_ = transform;
          body_basefootprint_ = transform;
      }

      // gait max velocities parameters
      if (!node_->get_parameter("gait_parameters.max_gait_linear_speed", max_gait_linear_speed_))
      {
          const char* message = "Parameter gait_parameters.max_gait_linear_speed was not found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      RCLCPP_INFO(node_->get_logger(), "Loaded gait_parameters.max_gait_linear_speed value is: %f [m/sec]", max_gait_linear_speed_);
      
      if (!node_->get_parameter("gait_parameters.max_gait_turning_speed", max_gait_turning_speed_))
      {
          const char* message = "Parameter gait_parameters.max_gait_turning_speed was not found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      RCLCPP_INFO(node_->get_logger(), "Loaded gait_parameters.max_gait_turning_speed value is: %f [rad/sec]", max_gait_turning_speed_);
  }

}  // penta_pod::kin::gait_generator
