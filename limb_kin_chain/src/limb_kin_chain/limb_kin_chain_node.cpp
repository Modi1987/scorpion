#include <rclcpp/executors.hpp>

// include librarries
#include "limb_kin_chain/limb_kin_chain.hpp"
#include "limb_kin_chain/limb_kin_chain_node.hpp"
#include "rclcpp/rclcpp.hpp"

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include <string>
#include <vector>
#define pi 3.141592

namespace penta_pod::kin::limb_kin_chain {

LimbNode::LimbNode(std::shared_ptr<LimbIKInterface> limb,
                   rclcpp::Node::SharedPtr node)
    : node_{node}, limb_(limb) {
  RCLCPP_INFO(node_->get_logger(), "Starting limb node");

  const auto &opts = node_->get_node_options();
  if (opts.use_intra_process_comms()) {
    RCLCPP_INFO(node_->get_logger(), ">> Intra-process comms is ENABLED");
  } else {
    RCLCPP_INFO(node_->get_logger(), ">> Intra-process comms is DISABLED");
  }

  this->declare_parameters();

  int dof = 3;
  if (node_->get_parameter("modified_dh.dof", dof)) {
    RCLCPP_INFO(node_->get_logger(), "limb dof loaded successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    rclcpp::shutdown();
  }
  std::vector<double> a;
  if (node_->get_parameter("modified_dh.a", a)) {
    RCLCPP_INFO(node_->get_logger(),
                "limb DH paramter (a) loaded successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    rclcpp::shutdown();
  }
  std::vector<double> d;
  if (node_->get_parameter("modified_dh.d", d)) {
    RCLCPP_INFO(node_->get_logger(),
                "limb DH paramter (d) loaded successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    rclcpp::shutdown();
  }
  std::vector<double> alfa;
  if (node_->get_parameter("modified_dh.alfa", alfa)) {
    RCLCPP_INFO(node_->get_logger(),
                "limb DH parameter (alfa) loaded successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    rclcpp::shutdown();
  }
  std::vector<double> eef_trans;
  if (node_->get_parameter("modified_dh.eef_trans", eef_trans)) {
    RCLCPP_INFO(node_->get_logger(), "limb eef_trans loaded successfully");
  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    rclcpp::shutdown();
  }
  // get initial q0
  if (node_->get_parameter("modified_dh.q0", q_state_)) {

  } else {
    RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    rclcpp::shutdown();
  }
  std::vector<double> q_max;
  if (node_->get_parameter("modified_dh.q_max", q_max)) {
    RCLCPP_INFO(node_->get_logger(), "max joints angles loaded successfully");
    RCLCPP_INFO(node_->get_logger(), "q_max size: %zu", q_max.size());
    std::stringstream ss;
    for (size_t i = 0; i < q_max.size(); i++) {
      ss << q_max[i] << " ";
    }
    ss << "\n";
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, can not load maximum joint limits q_max");
    rclcpp::shutdown();
  }
  std::vector<double> q_min;
  if (node_->get_parameter("modified_dh.q_min", q_min)) {
    RCLCPP_INFO(node_->get_logger(),
                "minimum joints angles loaded successfully");
    RCLCPP_INFO(node_->get_logger(), "q_min size: %zu", q_max.size());
    std::stringstream ss;
    for (size_t i = 0; i < q_min.size(); i++) {
      ss << q_min[i] << " ";
    }
    ss << "\n";
    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
  } else {
    RCLCPP_ERROR(node_->get_logger(),
                 "ERROR, can not load minimum joint limits q_min");
    rclcpp::shutdown();
  }

  q_target_ = std::vector<double>(dof, 0.0);

  limb_->init(dof, a, d, alfa, eef_trans, q_max, q_min);

  for (int i = 0; i < dof; i++) {
    std::string temp = "Joint_" + std::to_string(i);
    joints_names_.push_back(temp);
  }

  // clang-format off
  joint_setpoint_publisher_ =
      node_->create_publisher<sensor_msgs::msg::JointState>("joint_setpoints", 10);
  // clang-format on
  // you can publish initial joints states once
  sensor_msgs::msg::JointState msg;
  msg.name = joints_names_;
  msg.position = q_state_;
  joint_setpoint_publisher_->publish(msg);
  // following is for tcp position sub and joints setpoint pub
  joints_setpoint_msg_.name = joints_names_;
  joints_setpoint_msg_.position = q_target_;
  xyz_subscriber_ = node_->create_subscription<limb_msgs::msg::Pxyz>(
      "xyz_msg", 1, [this](const limb_msgs::msg::Pxyz &xyz_msg) -> void {
        double x = xyz_msg.x;
        double y = xyz_msg.y;
        double z = xyz_msg.z;
        // RCLCPP_INFO_STREAM(node_->get_logger(), "x, y, z received: " << x <<
        // y << z); get inverse kinematics
        bool success = limb_->get_ik(x, y, z, q_state_, q_target_);
        if (!success) {
          RCLCPP_ERROR(node_->get_logger(), "IK solver failed");
          return;
        }
        // update internal state
        for (size_t i = 0; i < q_target_.size(); i++) {
          q_state_[i] = q_target_[i]; // later must come from feedback
          joints_setpoint_msg_.position[i] = q_target_[i];
        }
        joint_setpoint_publisher_->publish(joints_setpoint_msg_);
      });
}

void LimbNode::declare_parameters() {
  // robot geometry
  node_->declare_parameter<int>("modified_dh.dof");
  node_->declare_parameter<std::vector<double>>("modified_dh.a",
                                                std::vector<double>());
  node_->declare_parameter<std::vector<double>>("modified_dh.d",
                                                std::vector<double>());
  node_->declare_parameter<std::vector<double>>("modified_dh.alfa",
                                                std::vector<double>());
  node_->declare_parameter<std::vector<double>>("modified_dh.eef_trans",
                                                std::vector<double>());
  node_->declare_parameter<std::vector<double>>("modified_dh.q0",
                                                std::vector<double>());
  node_->declare_parameter<std::vector<double>>("modified_dh.q_min",
                                                std::vector<double>());
  node_->declare_parameter<std::vector<double>>("modified_dh.q_max",
                                                std::vector<double>());
}

} // namespace penta_pod::kin::limb_kin_chain
