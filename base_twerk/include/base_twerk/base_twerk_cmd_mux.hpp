#ifndef BASE_TWERK_PUBLISHER_HPP_
#define BASE_TWERK_PUBLISHER_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp

// include messages
#include "geometry_msgs/msg/transform.hpp"

namespace penta_pod::kin::base_twerk_cmd_mux {

  class BaseTwerkCmdMux {
    private:
      rclcpp::Node::SharedPtr node_;
      rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr base_to_footprint_tarnsform_publisher_;
      rclcpp::TimerBase::SharedPtr timer_;

      geometry_msgs::msg::Transform body_basefootprint_transform_;
      int update_interval_millis_;

      void declare_parameters();
      void load_parameters();
      void timer_callback();

    public:
      explicit BaseTwerkCmdMux();
      void spin() {rclcpp::spin(node_);};
  };

} // namespace penta_pod::kin::base_twerk_cmd_mux

#endif  // BASE_TWERK_PUBLISHER_HPP_