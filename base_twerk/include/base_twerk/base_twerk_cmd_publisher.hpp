#ifndef BASE_TWERK_PUBLISHER_HPP_
#define BASE_TWERK_PUBLISHER_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include messages
#include "geometry_msgs/msg/transform.hpp"
#include "base_twerk_msgs/srv/base_pose_setpoint.hpp"

namespace penta_pod::kin::base_twerk_cmd_publisher {

  using Transform        = geometry_msgs::msg::Transform;
  using BasePoseSetpoint = base_twerk_msgs::srv::BasePoseSetpoint;
  
  class BaseTwerkCmdMux {
    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Publisher<Transform>::SharedPtr base_to_footprint_tarnsform_publisher_;
      rclcpp::Service<BasePoseSetpoint>::SharedPtr setpoint_service_;
      rclcpp::TimerBase::SharedPtr timer_;

      Transform body_basefootprint_transform_;
      Transform setpoint_body_basefootprint_transform_;
      int update_interval_millis_;

      void declare_parameters();
      void load_parameters();
      void timer_callback();
      void create_setpoint_service();

    public:
      explicit BaseTwerkCmdMux();
      void spin() {rclcpp::spin(node_);};
  };

} // namespace penta_pod::kin::base_twerk_cmd_publisher

#endif  // BASE_TWERK_PUBLISHER_HPP_