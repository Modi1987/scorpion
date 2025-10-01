#include "joints_aggregator/joints_aggregator.hpp"
#include <rclcpp/executors.hpp>

// include librarries
#include "rclcpp/rclcpp.hpp"

#include <vector>
// include messages
#include "sensor_msgs/msg/joint_state.hpp"

namespace penta_pod::kin::joints_aggregator {

JointsAggregator::JointsAggregator(rclcpp::Node::SharedPtr node)
    : node_{node} {
  RCLCPP_INFO(node_->get_logger(), "Starting joints aggregator node");
  
  const auto & opts = node_->get_node_options();
  if (opts.use_intra_process_comms()) {
    RCLCPP_INFO(node_->get_logger(), ">> Intra-process comms is ENABLED");
  } else {
    RCLCPP_INFO(node_->get_logger(), ">> Intra-process comms is DISABLED");
  }

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
      name_space_ + "limb" + std::to_string(i) + "/joint" + std::to_string(j);
      RCLCPP_INFO(node_->get_logger(), "joint state [%d] name is %s",
                  joints_count, temp.c_str());
      joints_states_names.push_back(temp);
    }
  }
  RCLCPP_INFO(node_->get_logger(), "total joints count is %d", joints_count);

  this->q_ = std::vector<double>(joints_count, 0.);
  this->previous_q_ = std::vector<double>(joints_count, 0.);

  joint_state_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_setpoints",
                                                            10);

  for (int i = 0; i < limbs_num_; i++) {
    std::string topic_string = "limb" + std::to_string(i) + "/joint_setpoints";
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

        // publish logic
        auto force_publish_on_startup = [this]() -> bool {
          if (publish_on_startup_counter_ <
              number_of_messages_forcely_published_on_startup_) {
            publish_on_startup_counter_++;
            return true;
          }
          return false;
        };

        bool force_publish = !publish_joints_only_on_value_change_;

        auto check_joints_value_change = [this]() -> bool {
          constexpr auto epsilon = 1e-10;
          auto total_angular_change_squared = 0.0;
          for (size_t i = 0; i < q_.size(); i++) {
            auto value = q_[i] - previous_q_[i];
            total_angular_change_squared += (value * value);
          }
          return (total_angular_change_squared > epsilon);
        };

        if (force_publish_on_startup()) {
          joint_state_publisher_->publish(msg);
        } else if (force_publish || check_joints_value_change()) {
          joint_state_publisher_->publish(msg);
        }

        for (size_t i = 0; i < q_.size(); i++) {
          previous_q_[i] = q_[i];
        }
      });
}

void JointsAggregator::declare_parameters() {
  // robot geometry
  node_->declare_parameter<std::string>("name_space", "");
  node_->declare_parameter<int>("limbs_num");
  node_->declare_parameter<std::vector<long int>>("joints_per_limb",
                                                  std::vector<long int>{});
  // joint_states publish rate (time interval)
  node_->declare_parameter<int>(
      "joints_aggregator.joints_update_interval_millis");
  // publish only on value change
  node_->declare_parameter<bool>(
      "joints_aggregator.publish_joints_only_on_value_change");
  // number of messages forcely published on startup
  node_->declare_parameter<int>(
      "joints_aggregator.number_of_messages_forcely_published_on_startup",
      0);
}

auto JointsAggregator::get_parameters() -> bool {
  // load name_space
  if (node_->get_parameter("name_space", name_space_)) {
    RCLCPP_INFO_STREAM(node_->get_logger(),
                        "name_space parameter loaded and equal to: "
                            << name_space_);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, can not load name_space parameter");
    return false;
  }
  // load limbs_num
  if (node_->get_parameter("limbs_num", limbs_num_)) {
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "(limbs_num) parameter loaded and equal to: " << limbs_num_);
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, can not load (limbs_num) parameter");
    return false;
  }
  // load joints_per_limb vector
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
  // sanity check
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
  // load joints_update_interval_millis
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
  // load publish_joints_only_on_value_change
  if (node_->get_parameter(
          "joints_aggregator.publish_joints_only_on_value_change",
          publish_joints_only_on_value_change_)) {
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "loaded joints_aggregator.publish_joints_only_on_value_change: "
            << publish_joints_only_on_value_change_);
  } else {
    RCLCPP_ERROR(
        node_->get_logger(),
        "ERROR, can not load "
        "joints_aggregator.publish_joints_only_on_value_change parameter");
    return false;
  }
  // load number_of_messages_forcely_published_on_startup_
  if (node_->get_parameter(
          "joints_aggregator.number_of_messages_forcely_published_on_startup",
          number_of_messages_forcely_published_on_startup_)) {
    RCLCPP_INFO_STREAM(
        node_->get_logger(),
        "loaded "
        "joints_aggregator.number_of_messages_forcely_published_on_startup: "
            << number_of_messages_forcely_published_on_startup_);
  } else {
    RCLCPP_ERROR(
        node_->get_logger(),
        "ERROR, can not load "
        "joints_aggregator.number_of_messages_forcely_published_on_startup "
        "parameter");
    return false;
  }

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
