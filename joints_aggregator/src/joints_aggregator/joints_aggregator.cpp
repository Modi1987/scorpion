#include "joints_aggregator/joints_aggregator.hpp"
#include <rclcpp/executors.hpp>

// include librarries
#include "rclcpp/rclcpp.hpp"     

#include <vector>
// include messages
#include "sensor_msgs/msg/joint_state.hpp"

namespace penta_pod::kin::joints_aggregator {

  JointsAggregator::JointsAggregator() : node_{rclcpp::Node::make_shared("joints_aggregator_node")}
  {
    RCLCPP_INFO(node_->get_logger(), "Starting joints aggregator node");
    this->declare_parameters();
    
    int limbs_num;
    if(node_->get_parameter("limbs_num", limbs_num)) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "(limbs_num)) parameter loaded and equal to: "<<limbs_num);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load (limbs_num) parameter");
    }

    int joints_per_limb;
    if(node_->get_parameter("joints_per_limb", joints_per_limb)) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "(joints_per_limb) parameter loaded and equal to: "<<joints_per_limb);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load (joints_per_limb) parameter");
    }

    int joints_count = 0;
    for(int i=0; i < limbs_num; i++) {
      for(int j = 0; j < joints_per_limb; j++) {
        joints_count = joints_count + 1;
        std::string temp = "limb" + std::to_string(i) + "/joint" + std::to_string(j);
        joints_states_names.push_back(temp);
      }
    }

    q = std::vector<double>(joints_count, 0.);
    
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    for(int i=0; i < limbs_num; i++) {
        std::string topic_string = "/limb" + std::to_string(i) + "/joint_state";
          limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_string, 1,
        [i, this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb(i, msg);
        }
      ));
    }
  }

  void JointsAggregator::declare_parameters(){
    // robot geometry
    node_->declare_parameter<int>("limbs_num");
    node_->declare_parameter<int>("joints_per_limb");
  }

  void JointsAggregator::on_joint_state_callback_limb(int limb_index, const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }

};  // penta_pod::kin::joints_aggregator
