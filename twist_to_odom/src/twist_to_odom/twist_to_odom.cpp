#include <rclcpp/executors.hpp>

// include librarries
#include "twist_to_odom/twist_to_odom.hpp"
#include "rclcpp/rclcpp.hpp"     


// include messages
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

#include "tf2_ros/transform_broadcaster.h"

#include <cmath>

#define pi 3.141592

namespace polypod::twist_to_odom {

  TwistToOdom::TwistToOdom() : node_{rclcpp::Node::make_shared("twist_to_odom_node")} {
    RCLCPP_INFO(node_->get_logger(), "Starting cmd_vel to odom node");

    odometry_publisher_ =  node_->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    cmd_vel_subscriber_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "feedback_cmd_vel", 1,
      [this](const geometry_msgs::msg::Twist& cmd_vel_msg) -> void {
        cmd_vel_ = cmd_vel_msg;
      });

      constexpr auto timer_period_millis = 100;
      timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(timer_period_millis),
        [this, timer_period_millis]() -> void {

          auto vx = cmd_vel_.linear.x;
          auto vy = cmd_vel_.linear.y;
          auto w = cmd_vel_.angular.z;

          constexpr auto millis_to_seconds = 1000.0;
          auto vx_dt = vx * (timer_period_millis / millis_to_seconds);
          auto vy_dt = vy * (timer_period_millis / millis_to_seconds);
          auto w_dt = w * (timer_period_millis / millis_to_seconds);
          theta_ += w_dt;

          auto dx = vx_dt * cos(theta_) - vy_dt * sin(theta_);
          auto dy = vx_dt * sin(theta_) + vy_dt * cos(theta_);

          state_.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
          state_.header.frame_id = "odom";
          state_.child_frame_id = "base_footprint";
          state_.pose.pose.position.x += dx;
          state_.pose.pose.position.y += dy;
          state_.pose.pose.position.z  = 0.0;
          state_.pose.pose.orientation.x = 0.0;
          state_.pose.pose.orientation.y = 0.0;
          state_.pose.pose.orientation.z = sin(theta_/2);
          state_.pose.pose.orientation.w = cos(theta_/2);
          
          // Populate the twist information
          state_.twist.twist.linear.x = vx;
          state_.twist.twist.linear.y = vy;
          state_.twist.twist.angular.z = w;

          state_.pose.covariance = {0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01};

          state_.twist.covariance = {0.01, 0, 0, 0, 0, 0,
                                    0, 0.01, 0, 0, 0, 0,
                                    0, 0, 0.01, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.01};


          odometry_publisher_->publish(state_);

          // publish transforms
          geometry_msgs::msg::TransformStamped transformStamped;
          transformStamped.header.stamp = state_.header.stamp;
          transformStamped.header.frame_id = "odom";
          transformStamped.child_frame_id = "base_footprint";

          // Set the translation
          transformStamped.transform.translation.x = state_.pose.pose.position.x;
          transformStamped.transform.translation.y = state_.pose.pose.position.y;
          transformStamped.transform.translation.z = state_.pose.pose.position.z;
          transformStamped.transform.rotation.x = state_.pose.pose.orientation.x;
          transformStamped.transform.rotation.y = state_.pose.pose.orientation.y;
          transformStamped.transform.rotation.z = state_.pose.pose.orientation.z;
          transformStamped.transform.rotation.w = state_.pose.pose.orientation.w;
          // Broadcast the transform
          // tf_broadcaster_->publish(transformStamped);
          // tf2_ros
          static auto br = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
          br->sendTransform(transformStamped);
        }
    );
  }

}  // namespace polypod::twist_to_odom
