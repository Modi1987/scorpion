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
    const double delta_t_milli = 10.;

    current_phase_ = 0.;
    for(int i = 0; i < feet_num_; i++) {
        auto topic_name = "limb" + std::to_string(i) + "/xyz_msg";
        xyz_publishers_.push_back(node_->create_publisher<limb_msgs::msg::Pxyz>(topic_name, 10));
        final_displacement_.push_back(geometry_msgs::msg::Point());
    }
    phase_shift_vec_ = init_phase_shift(feet_num_);

    cmd_vel_subscription_ = node_->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg){ cmd_vel_sub_callback(msg); });

    cmd_null_pos_subscription_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
        "null_space_pose", 10, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){ cmd_null_pos_sub_callback(msg); });

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

  void GaitGenerator::cmd_null_pos_sub_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // add to the initial displacement
    body_basefootprint_.translation.x = msg->pose.position.x;
    body_basefootprint_.translation.y = msg->pose.position.y;
    body_basefootprint_.translation.z = msg->pose.position.z;
    // consider the rotation absolute (initial rotation must be the identity)
    // body_basefootprint_.rotation. = msg->rotation;
    RCLCPP_INFO(node_->get_logger(), "Received nullspace translation from equilibrium (default): displacement.x=%.2f, displacement.y=%.2f, displacement.z=%.2f",
                body_basefootprint_.translation.x, body_basefootprint_.translation.y, body_basefootprint_.translation.z);
  }

  void GaitGenerator::update_phase(double delta_t_milli) {
    
    auto delta_t_sec = delta_t_milli/1000.;
    double w = 2.5;

    // check if cmd_vel is zero and feet near the equilibrium
    double vel_mag = std::sqrt(cmd_vel_.linear.x*cmd_vel_.linear.x + cmd_vel_.linear.y*cmd_vel_.linear.y);
    auto collective_xy_distance_from_equilibrium = 0.0;
    for(int i = 0; i < feet_num_; i++) {
      collective_xy_distance_from_equilibrium += std::abs(feet_pos_in_footprint_[i].x - init_feet_pos_in_footprint_[i].x) + 
      std::abs(feet_pos_in_footprint_[i].y - init_feet_pos_in_footprint_[i].y);
    }

    if ((collective_xy_distance_from_equilibrium < 0.005) && (vel_mag < 0.001)) {
      auto check_z_near_zero = current_phase_ - std::floor(current_phase_ / (2 * pi)) * 2 * pi;
      if (check_z_near_zero < w*delta_t_sec + 0.001)
      {
        w = 0.0;
        current_phase_ = std::floor(current_phase_ / (2 * pi)) * 2 * pi;
      }
    }
    current_phase_ = current_phase_ + w*delta_t_sec;
  }

  void GaitGenerator::update_feet_positions(double delta_t_milli) {

    auto delta_t_sec = delta_t_milli/1000.;
    
    double dx = cmd_vel_.linear.x*delta_t_sec;
    double dy = cmd_vel_.linear.y*delta_t_sec;
    double d_theta = cmd_vel_.angular.z*delta_t_sec;
    double b = 0.05;
    
    for(int i = 0; i < feet_num_; i++) {
        auto temp = foot_pos_z_generator(b,current_phase_,phase_shift_vec_[i],feet_num_);
        if(temp==0.) {
            double x = feet_pos_in_footprint_[i].x;
            double y = feet_pos_in_footprint_[i].y;
            feet_pos_in_footprint_[i].x = x + dx - d_theta*y;
            feet_pos_in_footprint_[i].y = y + dy + d_theta*x;
            feet_pos_in_footprint_[i].z = 0.;
            final_displacement_[i].x = feet_pos_in_footprint_[i].x - init_feet_pos_in_footprint_[i].x;
            final_displacement_[i].y = feet_pos_in_footprint_[i].y - init_feet_pos_in_footprint_[i].y;
        } else {
            feet_pos_in_footprint_[i].x = init_feet_pos_in_footprint_[i].x + 
                                            foot_pos_xy_generator(current_phase_, phase_shift_vec_[i], final_displacement_[i].x, feet_num_);
            feet_pos_in_footprint_[i].y = init_feet_pos_in_footprint_[i].y + 
                                            foot_pos_xy_generator(current_phase_, phase_shift_vec_[i], final_displacement_[i].y, feet_num_);
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

  void GaitGenerator::timer_callback(double delta_t_milli){
    update_phase(delta_t_milli);
    update_feet_positions(delta_t_milli);
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
    // Helper function to load a parameter and throw error if not found
    auto load_param = [this](const std::string &param_name, auto &param_value, const std::string &error_message) {
        if (!node_->get_parameter(param_name, param_value)) {
            RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
            throw std::runtime_error(error_message);
        }
    };

    // Helper function to validate vector size
    auto validate_vector_size = [this](const std::vector<double> &vec, int expected_size, const std::string &error_message) {
        if (static_cast<int>(vec.size()) != expected_size) {
            RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
            throw std::runtime_error(error_message);
        }
    };

    // Helper function to create a transform from values
    auto array_to_transform = [this](const std::vector<double> &transforms_vec, int frame_index) -> geometry_msgs::msg::Transform {
        auto minimal_required_size = 7*frame_index+7;
        if(transforms_vec.size() < static_cast<size_t>(minimal_required_size)) {
            auto error_message = "Error, can not create trasform from array at index " + std::to_string(frame_index) + " since minimal required size is less than " + std::to_string(minimal_required_size);
            RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
            throw std::runtime_error(error_message);
        }
        geometry_msgs::msg::Transform transform;
        auto start_index = frame_index*7;
        transform.translation.x = transforms_vec[start_index + 0];
        transform.translation.y = transforms_vec[start_index + 1];
        transform.translation.z = transforms_vec[start_index + 2];
        transform.rotation.x = transforms_vec[start_index + 3];
        transform.rotation.y = transforms_vec[start_index + 4];
        transform.rotation.z = transforms_vec[start_index + 5];
        transform.rotation.w = transforms_vec[start_index + 6];
        return transform;
    };

    // Load limbs number
    load_param("limbs_num", feet_num_, "No limbs_num parameter found.");
    RCLCPP_INFO(node_->get_logger(), "Loaded limbs_num value is: %d", feet_num_);

    // Load and validate leg-body transforms
    std::vector<double> transform_params;
    load_param("legs_body_transforms", transform_params, "No transform parameters found.");
    validate_vector_size(transform_params, feet_num_ * 7, 
        "Invalid transform parameter size, expected " + std::to_string(feet_num_) + "x7 elements.");
    
    for (int frame_index = 0; frame_index < feet_num_; ++frame_index) {
        auto transform = array_to_transform(transform_params, frame_index);
        legs_body_transforms_.emplace_back(transform);

        std::string message = "Shoulder [" + std::to_string(frame_index) + "] transform in body frame is: [ ";
        for (int i = 0; i < 7; ++i) {
            message += double_to_string_formatted(transform_params[i + 7 * frame_index], 4) + " ";
        }
        message += "]";
        RCLCPP_INFO(node_->get_logger(), message.c_str());
    }

    // Load and validate initial feet positions
    std::vector<double> init_feet_pos_params;
    load_param("init_feet_pos_in_basefootprint", init_feet_pos_params, "No feet position parameters found.");
    validate_vector_size(init_feet_pos_params, feet_num_ * 3,
        "Invalid feet position parameter size, expected " + std::to_string(feet_num_) + "x3 elements.");
    
    for (int count = 0; count < feet_num_; ++count) {
        geometry_msgs::msg::Point xyz;
        xyz.x = init_feet_pos_params[0 + count * 3];
        xyz.y = init_feet_pos_params[1 + count * 3];
        xyz.z = init_feet_pos_params[2 + count * 3];
        feet_pos_in_footprint_.emplace_back(xyz);
        init_feet_pos_in_footprint_.emplace_back(xyz);
    }

    // Load and validate initial body-to-basefootprint transform
    std::vector<double> body_basefootprint_params;
    load_param("init_body_basefootprint_transform", body_basefootprint_params,
        "No init transform body to basefootprint found.");
    validate_vector_size(body_basefootprint_params, 7,
        "Size of init_body_basefootprint_transform must be 7 (3 for position followed by 4 quaternion).");
    
    auto transform = array_to_transform(body_basefootprint_params, 0);
    body_basefootprint_ = transform;

    // Load gait parameters
    load_param("gait_parameters.max_gait_linear_speed", max_gait_linear_speed_,
        "Parameter gait_parameters.max_gait_linear_speed was not found.");
    RCLCPP_INFO(node_->get_logger(), "Loaded gait_parameters.max_gait_linear_speed value is: %f [m/sec]", max_gait_linear_speed_);

    load_param("gait_parameters.max_gait_turning_speed", max_gait_turning_speed_,
        "Parameter gait_parameters.max_gait_turning_speed was not found.");
    RCLCPP_INFO(node_->get_logger(), "Loaded gait_parameters.max_gait_turning_speed value is: %f [rad/sec]", max_gait_turning_speed_);
 }

}  // penta_pod::kin::gait_generator
