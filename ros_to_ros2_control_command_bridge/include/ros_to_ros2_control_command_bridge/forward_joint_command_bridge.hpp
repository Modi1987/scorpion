#ifndef ROS2_CONTROL_FORWARD_JOINT_COMMAND_BRIDGE_HPP
#define ROS2_CONTROL_FORWARD_JOINT_COMMAND_BRIDGE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace ros2_to_ros2_control_bridge
{
  class ForwardJointCommandControlBridge
  {
      
  public:
    ForwardJointCommandControlBridge();
    ~ForwardJointCommandControlBridge();
    
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void publishJointCommands();

    void spin()
    {
      rclcpp::spin(node_);
    }

  private:
      rclcpp::Node::SharedPtr node_;

      rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_publisher_;
      rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;

      std::vector<double> joint_angles_at_initial_pose_degree_;
      std::vector<double> actuator_angles_at_initial_pose_degree_;
      std::vector<double> dir_;
      
      void declareParameters();
      void loadParameters();
  };
}

#endif  // ROS2_CONTROL_FORWARD_JOINT_COMMAND_BRIDGE_HPP