#include <rclcpp/executors.hpp>

// include librarries
#include "limb_kin_chain/limb_kin_chain.hpp"  
#include "limb_kin_chain/limb_kin_chain_node.hpp"  
#include "rclcpp/rclcpp.hpp"     

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include <vector>
#include <string>
#define pi 3.141592

namespace penta_pod::kin::limb_kin_chain {

  LimbNode::LimbNode(Limb limb) : node_{rclcpp::Node::make_shared("limb_node")}, limb_(std::make_shared<Limb>(limb))  {
    RCLCPP_INFO(node_->get_logger(), "Starting limb node");
    this->declare_parameters();
    
    int dof;
    if(node_->get_parameter("modified_dh.dof", dof)) {
      RCLCPP_INFO(node_->get_logger(), "limb dof loaded successfully");  
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    }
    std::vector<double> a;
    if(node_->get_parameter("modified_dh.a", a)) {
      RCLCPP_INFO(node_->get_logger(), "limb DH paramter (a) loaded successfully");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    }
    std::vector<double> d;
    if(node_->get_parameter("modified_dh.d", d)) {
      RCLCPP_INFO(node_->get_logger(), "limb DH paramter (d) loaded successfully");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    }
    std::vector<double> alfa;
    if(node_->get_parameter("modified_dh.alfa", alfa)) {
      RCLCPP_INFO(node_->get_logger(), "limb DH parameter (alfa) loaded successfully");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    }
    std::vector<double> eef_trans;
    if(node_->get_parameter("modified_dh.eef_trans", eef_trans)) {
      RCLCPP_INFO(node_->get_logger(), "limb eef_trans loaded successfully");
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
    }
    // get initial q0
    if(node_->get_parameter("modified_dh.q0", q_state)) {

    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load dof parameter");
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
        RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load maximum joint limits q_max");
    }
    std::vector<double> q_min;
    if(node_->get_parameter("modified_dh.q_min", q_min)) {
      RCLCPP_INFO(node_->get_logger(), "minimum joints angles loaded successfully");
      RCLCPP_INFO(node_->get_logger(), "q_min size: %zu", q_max.size());
      std::stringstream ss;
      for (size_t i = 0; i < q_min.size(); i++) {
          ss << q_min[i] << " ";
      }
      ss << "\n";
      RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    } else {
      RCLCPP_ERROR(node_->get_logger(), "ERROR, can not load minimum joint limits q_min");
    }

    limb_->init(dof, a, d, alfa, eef_trans, q_max, q_min);
     
    for(int i = 0; i < dof; i++) {
      std::string temp = "Joint_" + std::to_string(i);
      joints_names.push_back(temp);
    }
    
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
    // you can publish initial joints states once
    sensor_msgs::msg::JointState msg;
    msg.name = joints_names;
    msg.position = q_state;
    joint_state_publisher_->publish(msg);
    // following is the tcp position subscriber
    xyz_subscriber_ = node_->create_subscription<limb_msgs::msg::Pxyz>(
      "xyz_msg", 1, 
      [this](const limb_msgs::msg::Pxyz& xyz_msg) -> void {
          double x = xyz_msg.x;
          double y = xyz_msg.y;
          double z = xyz_msg.z;
          RCLCPP_INFO_STREAM(node_->get_logger(), "x, y, z received: " << x << y << z);
          // get inverse kinematics
          std::vector<double> q = limb_->get_ik(x, y, z, q_state);
          // update internal state
          for(size_t i=0; i<q.size(); i++)q_state[i]=q[i];
          // create and publish the message
          sensor_msgs::msg::JointState msg;
          msg.name = joints_names;
          msg.position = q;
          joint_state_publisher_->publish(msg);
      }
    );
  }

  void LimbNode::declare_parameters(){
    // robot geometry
    node_->declare_parameter<int>("modified_dh.dof");
    node_->declare_parameter<std::vector<double>>("modified_dh.a", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.d", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.alfa", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.eef_trans", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.q0", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.q_min", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.q_max", std::vector<double>());
  }

}  // penta_pod::kin::limb_kin_chain
