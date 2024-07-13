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
    t_ = 0.;
    sign_ = 1.0;
    dof_ = 5;
    foot_count_ = 0;
    for(int i = 0; i < dof_; i++) {
        auto topic_name = "limb" + std::to_string(i) + "/xyz_msg";
        xyz_publishers_.push_back(node_->create_publisher<limb_msgs::msg::Pxyz>(topic_name, 10));
    }

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(delta_t_milli)),
        [this, delta_t_milli]() { timer_callback(delta_t_milli); });
  }

  void GaitGenerator::timer_callback(double delta_t_milli){
      t_ = t_ + delta_t_milli/1000.;
      double w = 1.2;
      double r = 0.05;
      auto temp = sign_*r*sin(w*t_);
      if(temp<0.){
        foot_count_++;
        if(foot_count_ == dof_) {
          foot_count_=0;
        }
        sign_ = -sign_;
        temp = -temp;
      }

      for(int i = 0; i < dof_; i++) {
          auto foot_pos_in_footprint = feet_pos_in_footprint_[i];
          if(i == foot_count_) {
              foot_pos_in_footprint.z += temp;
          }

          if (i < static_cast<int>(legs_body_transforms_.size())) {
            auto  point = applyInverseTransform(foot_pos_in_footprint, legs_body_transforms_[i]);
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
    node_->declare_parameter<std::vector<double>>("legs_body_transforms", std::vector<double>{});
    node_->declare_parameter<std::vector<double>>("init_feet_pos_in_base", std::vector<double>{});
  }

  
  void GaitGenerator::load_parameters() {
      std::vector<double> transform_params;
      if (!node_->get_parameter("legs_body_transforms", transform_params))
      {
          const char* message = "No transform parameters found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      if (transform_params.size() != 5 * 7)
      {
          const char* message = "Invalid transform parameter size, expected 5x7 elements.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      for (int count = 0; count < 5; count++)
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
      }

      std::vector<double> init_feet_pos_params;
      if (!node_->get_parameter("init_feet_pos_in_base", init_feet_pos_params))
      {
          const char* message = "No feet position parameters found.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      if (init_feet_pos_params.size() != 5 * 3)
      {
          const char* message = "Invalid feet position parameter size, expected 5x3 elements.";
          RCLCPP_ERROR(node_->get_logger(), message);
          throw std::runtime_error(message);
      }
      for (int count = 0; count < 7; count++)
      {
          geometry_msgs::msg::Point xyz;
          xyz.x = init_feet_pos_params[0 + count * 3];
          xyz.y = init_feet_pos_params[1 + count * 3];
          xyz.z = init_feet_pos_params[2 + count * 3];
          feet_pos_in_footprint_.emplace_back(xyz);
      }
  }

}  // penta_pod::kin::gait_generator
