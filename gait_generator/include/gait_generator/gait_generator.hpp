#ifndef PENTA_POD_KIN_GAIT_GENERATOR_HPP_
#define PENTA_POD_KIN_GAIT_GENERATOR_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include "geometry_msgs/msg/transform.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


namespace penta_pod::kin::gait_generator {
  
  class GaitGenerator {
    private:
      rclcpp::Node::SharedPtr node_;
      std::vector<double> q_state;

      double t_, sign_;
      int dof_, foot_count_;

      std::vector<geometry_msgs::msg::Transform> legs_body_transforms_;
      geometry_msgs::msg::Transform body_base_;      

      std::vector<geometry_msgs::msg::Point> feet_pos_in_footprint_;

      std::vector<rclcpp::Publisher<limb_msgs::msg::Pxyz>::SharedPtr> xyz_publishers_;
      rclcpp::TimerBase::SharedPtr timer_;

      void declare_parameters();
      void load_parameters();
      void timer_callback(double delta_t_milli);

      geometry_msgs::msg::Quaternion invertQuaternion(const geometry_msgs::msg::Quaternion &q)
      {
          geometry_msgs::msg::Quaternion q_inv;
          q_inv.x = -q.x;
          q_inv.y = -q.y;
          q_inv.z = -q.z;
          q_inv.w = q.w;
          return q_inv;
      }

      geometry_msgs::msg::Point applyInverseTransform(
          const geometry_msgs::msg::Point &point, 
          const geometry_msgs::msg::Transform &transform)
      {
          geometry_msgs::msg::Quaternion q_inv = invertQuaternion(transform.rotation);
          tf2::Quaternion tf2_q_inv(q_inv.x, q_inv.y, q_inv.z, q_inv.w);

          geometry_msgs::msg::Point p_transformed;
          p_transformed.x = point.x - transform.translation.x;
          p_transformed.y = point.y - transform.translation.y;
          p_transformed.z = point.z - transform.translation.z;

          tf2::Vector3 tf2_point(p_transformed.x, p_transformed.y, p_transformed.z);
          tf2::Vector3 tf2_point_rotated = tf2::quatRotate(tf2_q_inv, tf2_point);

          geometry_msgs::msg::Point result;
          result.x = tf2_point_rotated.x();
          result.y = tf2_point_rotated.y();
          result.z = tf2_point_rotated.z();

          return result;
      }


    public:
      explicit GaitGenerator();
      void spin() {rclcpp::spin(node_);};
  };

} // namespace penta_pod::kin::gait_generator

#endif  // PENTA_POD_KIN_GAIT_GENERATOR_HPP_