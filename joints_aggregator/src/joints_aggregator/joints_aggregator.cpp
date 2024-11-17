#include "joints_aggregator/joints_aggregator.hpp"
#include <rclcpp/executors.hpp>

// include librarries
#include "rclcpp/rclcpp.hpp"     

#include <vector>
// include messages
#include "sensor_msgs/msg/joint_state.hpp"

namespace penta_pod::kin::joints_aggregator {

  const int DEAFAULT_UPDATE_INTERVAL_MILLIS{25};

  JointsAggregator::JointsAggregator() : node_{rclcpp::Node::make_shared("joints_aggregator_node")}
  {
    RCLCPP_INFO(node_->get_logger(), "Starting joints aggregator node");
    this->declare_parameters();
    if (!this->get_parameters()) {
      rclcpp::shutdown();
      return;
    }
    
    int joints_count = 0;
    for(int i=0; i < limbs_num_; i++) {
      for(int j = 0; j < joints_per_limb_; j++) {
        joints_count = joints_count + 1;
        std::string temp = "limb" + std::to_string(i) + "/joint" + std::to_string(j);
        joints_states_names.push_back(temp);
      }
    }

    this->q_ = std::vector<double>(joints_count, 0.);
    
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

    for(int i=0; i < limbs_num_; i++) {
        std::string topic_string = "/limb" + std::to_string(i) + "/joint_state";
          limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        topic_string, 10,
        [i, this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb(i, msg);
        }
      ));
    }

    timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(update_interval_millis_)),
        [this]() { 
          sensor_msgs::msg::JointState msg;
          msg.header.stamp = rclcpp::Clock(RCL_ROS_TIME).now();
          msg.name = joints_states_names;
          {
            std::lock_guard<std::mutex> lock(q_mutex_);
            msg.name = joints_states_names;
            msg.position = q_;
          }
          joint_state_publisher_->publish(msg);
        }        
      );
  }

  void JointsAggregator::declare_parameters(){
    // robot geometry
    node_->declare_parameter<int>("limbs_num");
    node_->declare_parameter<int>("joints_per_limb");
    // joint_states publish rate (time)
    node_->declare_parameter<int>("update_interval_millis", DEAFAULT_UPDATE_INTERVAL_MILLIS); // here DEAFAULT_UPDATE_INTERVAL_MILLIS is default value
  }

  auto JointsAggregator::get_parameters() -> bool {
    if(node_->get_parameter("limbs_num", limbs_num_)) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "(limbs_num)) parameter loaded and equal to: " << limbs_num_);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load (limbs_num) parameter");
      return false;
    }

    if(node_->get_parameter("joints_per_limb", joints_per_limb_)) {
      RCLCPP_INFO_STREAM(node_->get_logger(), "(joints_per_limb) parameter loaded and equal to: " << joints_per_limb_);
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load (joints_per_limb) parameter");
      return false;
    }
    
    if(!node_->get_parameter("update_interval_millis", update_interval_millis_)) {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load update_interval_millis defaulting to %d milliseconds", DEAFAULT_UPDATE_INTERVAL_MILLIS);
      update_interval_millis_ = DEAFAULT_UPDATE_INTERVAL_MILLIS;
    }
    if (update_interval_millis_ <= 0.0) {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, specified update_interval_millis %d can not be zero nor negative!", update_interval_millis_);
      RCLCPP_ERROR(node_->get_logger(), "ERROR, defaulting update_interval_millis to %d milliseconds", DEAFAULT_UPDATE_INTERVAL_MILLIS);
      update_interval_millis_ = DEAFAULT_UPDATE_INTERVAL_MILLIS;
    }
    auto rate = 1000.0 / update_interval_millis_;
    RCLCPP_INFO_STREAM(node_->get_logger(), "/joints_states: publish rate is: " << rate << " Hz");

    return true;
  }

  void JointsAggregator::on_joint_state_callback_limb(int limb_index, const sensor_msgs::msg::JointState& joint_state) {
    int index_start = limb_index * joints_per_limb_;
    std::lock_guard<std::mutex> lock(q_mutex_);
    for (int j = 0; j < joints_per_limb_; ++j) {
      if (static_cast<size_t>(j) < joint_state.position.size()) {
        q_[index_start + j] = joint_state.position[j];
      } else {
        RCLCPP_WARN(node_->get_logger(), "Received joint state with fewer positions than expected.");
      }
    }
  }

};  // penta_pod::kin::joints_aggregator
