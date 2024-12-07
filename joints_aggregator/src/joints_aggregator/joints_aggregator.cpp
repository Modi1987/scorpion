#include "joints_aggregator/joints_aggregator.hpp"
#include <rclcpp/executors.hpp>

// include librarries
#include "rclcpp/rclcpp.hpp"

#include <vector>
// include messages
#include "sensor_msgs/msg/joint_state.hpp"

namespace penta_pod::kin::joints_aggregator {

JointsAggregator::JointsAggregator()
    : node_{rclcpp::Node::make_shared("joints_aggregator_node")} {
  RCLCPP_INFO(node_->get_logger(), "Starting joints aggregator node");
  this->declare_parameters();
  if (!this->get_parameters()) {
    rclcpp::shutdown();
    return;
  }

  int joints_count = 0;
  for (int i = 0; i < limbs_num_; i++) {
    for (int j = 0; j < joints_per_limb_[i]; j++) {
      joints_count = joints_count + 1;
      std::string temp =
          "limb" + std::to_string(i) + "/joint" + std::to_string(j);
      RCLCPP_INFO(node_->get_logger(), "joint state [%d] name is %s",
                  joints_count, temp.c_str());
      joints_states_names.push_back(temp);
    }
  }
  RCLCPP_INFO(node_->get_logger(), "total joints count is %d", joints_count);

  this->q_ = std::vector<double>(joints_count, 0.);

  joint_state_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

  for (int i = 0; i < limbs_num_; i++) {
    std::string topic_string = "limb" + std::to_string(i) + "/joint_state";
    limb_joints_subscriber_.push_back(
        node_->create_subscription<sensor_msgs::msg::JointState>(
            topic_string, 10,
            [i, this](const sensor_msgs::msg::JointState &msg) -> void {
              this->on_joint_state_callback_limb(i, msg);
            }));
    RCLCPP_INFO(node_->get_logger(),
                "subscriber is created, listining on topic name %s",
                topic_string.c_str());
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
      });
}

void JointsAggregator::declare_parameters() {
  // robot geometry
  node_->declare_parameter<int>("limbs_num");
  node_->declare_parameter<std::vector<long int>>("joints_per_limb",
                                                  std::vector<long int>{});
  // joint_states publish rate (time interval)
  node_->declare_parameter<int>(
      "joints_aggregator.joints_update_interval_millis");
}

auto JointsAggregator::get_parameters() -> bool {
  if (node_->get_parameter("limbs_num", limbs_num_)) {
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "(limbs_num) parameter loaded and equal to: " << limbs_num_);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, can not load (limbs_num) parameter");
    return false;
  }

  if (node_->get_parameter("joints_per_limb", joints_per_limb_)) {
    std::string formatted_values = "[";
    for (auto &value : joints_per_limb_)
      formatted_values += " " + std::to_string(value);
    formatted_values += " ]";
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       "(joints_per_limb) parameter loaded and equal to: "
                           << formatted_values);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, can not load (joints_per_limb) parameter");
    return false;
  }
  if (joints_per_limb_.size() != static_cast<size_t>(limbs_num_)) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, value of parameter (limbs_num) is not equal to the "
                 "size of vector (joints_per_limb)!");
    return false;
  } else {
    RCLCPP_INFO(node_->get_logger(),
                "Parameter (limbs_num) and size of vector (joints_per_limb) "
                "comply with a value %d",
                limbs_num_);
  }

  if (!node_->get_parameter("joints_aggregator.joints_update_interval_millis",
                            update_interval_millis_)) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "ERROR, can not load joints_aggregator.joints_update_interval_millis");
    return false;
  }
  if (update_interval_millis_ <= 0.0) {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, specified update_interval_millis %d can not be zero "
                 "nor negative!",
                 update_interval_millis_);
    return false;
  }
  RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "loaded joints_aggregator.joints_update_interval_millis is: "
          << update_interval_millis_ << " milliseconds");
  auto rate = 1000.0 / update_interval_millis_;
  RCLCPP_INFO_STREAM(node_->get_logger(),
                     "/joints_states: publish rate is: " << rate << " Hz");

  return true;
}

void JointsAggregator::on_joint_state_callback_limb(
    int limb_index, const sensor_msgs::msg::JointState &joint_state) {
  int index_start = 0;
  for (int i = 0; i < limb_index; i++) {
    index_start = index_start + joints_per_limb_[i];
  }
  std::lock_guard<std::mutex> lock(q_mutex_);
  for (int j = 0; j < joints_per_limb_[limb_index]; ++j) {
    if (static_cast<size_t>(j) < joint_state.position.size()) {
      q_[index_start + j] = joint_state.position[j];
    } else {
      RCLCPP_WARN(node_->get_logger(),
                  "Received joint state with fewer positions than expected.");
    }
  }
}

}; // namespace penta_pod::kin::joints_aggregator
