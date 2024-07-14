#ifndef PENTA_POD_KIN_GAIT_GENERATOR_HPP_
#define PENTA_POD_KIN_GAIT_GENERATOR_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>

namespace penta_pod::kin::gait_generator {

  class GaitGenerator {
    private:
      rclcpp::Node::SharedPtr node_;
      std::vector<double> q_state;

      double current_phase_;
      int feet_num_;

      std::vector<geometry_msgs::msg::Transform> legs_body_transforms_;
      geometry_msgs::msg::Transform body_basefootprint_;      

      std::vector<geometry_msgs::msg::Point> feet_pos_in_footprint_; // current feet pos
      std::vector<geometry_msgs::msg::Point> init_feet_pos_in_footprint_; // feet pos at equilibrium
      std::vector<geometry_msgs::msg::Point> final_displacement_; // feet pos displacement from equilbrium
      std::vector<double> phase_shift_vec_;

      std::vector<rclcpp::Publisher<limb_msgs::msg::Pxyz>::SharedPtr> xyz_publishers_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
      rclcpp::TimerBase::SharedPtr timer_;
      geometry_msgs::msg::Twist cmd_vel_{};

      void declare_parameters();
      void load_parameters();
      void timer_callback(double delta_t_milli);
      void cmd_vel_sub_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

      // move feet up (z up) calculation
      double stepFunOneLegOff_2(double b, double q, double phase_shift, int n) {
        q = q + phase_shift; // add the phase
        q = q - std::floor(q / (2 * M_PI)) * 2 * M_PI; // remove multiples of 2*pi (resulting q is always less than 2*pi)
        double epsilon = M_PI / n;

        if (q < ((n - 1) * 2 * epsilon)) { // interval where feet is on ground
            return 0.0;
        } else if (q < 2 * M_PI) { // when feet is off the ground
            double u = M_PI * (q - (n - 1) * 2 * epsilon) / (2 * epsilon);
            return b * std::sin(u);
        } else {
            return 0.0;
        }
      }

      // moving foot back to equilbrium
      double stepFunOneLegOff_3(double q, double phase_shift, double delta_xFinal, int legsNum) {
          q = q + phase_shift;
          q = q - std::floor(q / (2 * M_PI)) * 2 * M_PI;
          double epsilon = M_PI / legsNum;

          if (q < 2 * (M_PI - epsilon)) {
              return delta_xFinal;
          } else if (q < 2 * M_PI) {
              double u = q - 2 * epsilon * (legsNum - 1);
              double alfa = delta_xFinal / (2 * epsilon);
              return delta_xFinal - alfa * u;
          } else {
              return 0.0;
          }
      }

      auto init_phase_shift(int feet_num) -> std::vector<double>  {
        std::vector<double> phase_shift_vec(feet_num);
        for (int i = 0; i < feet_num; ++i) {
            phase_shift_vec[i] = i * (2 * M_PI / feet_num);
        }
        return phase_shift_vec;
      }

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