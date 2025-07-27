#ifndef POLYPOD_CMD_VEL_TO_ODOM_HPP_
#define POLYPOD_CMD_VEL_TO_ODOM_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include messages
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance.hpp"
#include "geometry_msgs/msg/twist_with_covariance.hpp"

#include "tf2_ros/transform_broadcaster.h"

namespace polypod::cmd_vel_to_odom {
  class CmdVelToOdom {
  public:
    explicit CmdVelToOdom();
    void spin() { rclcpp::spin(node_); }

  private:
    rclcpp::Node::SharedPtr node_;

    double theta_{};

    geometry_msgs::msg::Twist cmd_vel_{};
    nav_msgs::msg::Odometry state_{};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    void declare_parameters();
  };

}  // namespace polypod::cmd_vel_to_odom

#endif  // POLYPOD_CMD_VEL_TO_ODOM_HPP_