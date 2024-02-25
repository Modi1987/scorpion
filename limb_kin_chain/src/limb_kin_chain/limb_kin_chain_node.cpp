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
    node_->get_parameter("modified_dh.dof", dof);
    std::vector<double> a;
    node_->get_parameter("modified_dh.a", a);
    std::vector<double> d;
    node_->get_parameter("modified_dh.d", d);
    std::vector<double> alfa;
    node_->get_parameter("modified_dh.alfa", alfa);
    std::vector<double> eef_trans;
    node_->get_parameter("modified_dh.eef_trans", eef_trans);
    limb_->init(dof, a, d, alfa, eef_trans);
     
    for(int i = 0; i < dof; i++) {
      std::string temp = "Joint_" + std::to_string(i);
      joints_names.push_back(temp);
    }
    
    joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("joint_state", 10);
    
    xyz_subscriber_ = node_->create_subscription<limb_msgs::msg::Pxyz>(
      "xyz_msg", 1, 
      [this](const limb_msgs::msg::Pxyz& xyz_msg) -> void {
          double x = xyz_msg.x;
          double y = xyz_msg.y;
          double z = xyz_msg.z;
          RCLCPP_INFO_STREAM(node_->get_logger(), "x, y, z received: " << x << y << z);
          std::vector<double> q = limb_->get_ik(x, y, z);
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
  }

}  // penta_pod::kin::limb_kin_chain
