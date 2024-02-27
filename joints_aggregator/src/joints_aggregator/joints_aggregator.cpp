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

    // add callbacks according to limbs_num (hard-coded)
    // ToDo, try to use templates or something to create the subscribers
    // in runtime while still have access to (q) from the class
    if(limbs_num>0){
          limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb0/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb0(msg);
        }
      )); 
    }

    if(limbs_num>1){
      limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb1/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb1(msg);
        }
      )); 
    }

    if(limbs_num>2){  
      limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb2/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb2(msg);
        }
      )); 
    }

    if(limbs_num>3){
      limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb3/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb3(msg);
        }
      ));
    }

    if(limbs_num>4){
      limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb4/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb4(msg);
        }
      ));
    }

    if(limbs_num>5){
      limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb5/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb5(msg);
        }
      ));
    }

    if(limbs_num>6){
      limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb6/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb6(msg);
        }
      ));
    }

    if(limbs_num>7){
      limb_joints_subscriber_.push_back(node_->create_subscription<sensor_msgs::msg::JointState>(
        "/limb7/joint_state", 1, 
        [this](const sensor_msgs::msg::JointState& msg) -> void {
            this->on_joint_state_callback_limb7(msg);
        }
      ));
    }
  }

  void JointsAggregator::declare_parameters(){
    // robot geometry
    node_->declare_parameter<int>("limbs_num");
    node_->declare_parameter<int>("joints_per_limb");
  }

  void JointsAggregator::on_joint_state_callback_limb0(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 0;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }

  void JointsAggregator::on_joint_state_callback_limb1(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 1;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }


  void JointsAggregator::on_joint_state_callback_limb2(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 2;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }

  void JointsAggregator::on_joint_state_callback_limb3(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 3;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }

  void JointsAggregator::on_joint_state_callback_limb4(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 4;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }

  void JointsAggregator::on_joint_state_callback_limb5(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 5;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }

  void JointsAggregator::on_joint_state_callback_limb6(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 6;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }

  void JointsAggregator::on_joint_state_callback_limb7(const sensor_msgs::msg::JointState& joint_state) {
    sensor_msgs::msg::JointState msg;
    const int limb_index = 7;
    int index_start = limb_index*3;
    this->q[index_start+0] = joint_state.position[0];
    this->q[index_start+1] = joint_state.position[1];
    this->q[index_start+2] = joint_state.position[2];
    msg.name = joints_states_names;
    msg.position = q;
    joint_state_publisher_->publish(msg);
  }


};  // penta_pod::kin::joints_aggregator
