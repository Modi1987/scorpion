#include <rclcpp/executors.hpp>

// include librarries 
#include "test_foot_pos/test_gait.hpp"  
#include "rclcpp/rclcpp.hpp"     

// include messages
#include "limb_msgs/msg/pxyz.hpp"
#include <vector>
#include <string>
#define pi 3.141592

namespace penta_pod::kin::test_gait {

  TestGait::TestGait() : node_{rclcpp::Node::make_shared("test_gait_node")}
  {
    RCLCPP_INFO(node_->get_logger(), "Starting test_gait_node");
    //this->declare_parameters();
    //this->load_paramters();
    const double delta_t_milli = 50.;
    t_ = 0.;
    sign_ = 1.0;
    dof_ = 5;
    foot_count_ = 0;
    for(int i = 0; i < dof_; i++) {
        auto topic_name = "limb" + std::to_string(i) + "/xyz_msg";
        xyz_publishers_.push_back(node_->create_publisher<limb_msgs::msg::Pxyz>(topic_name, 10));
    }
    timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(delta_t_milli)),
      [this, delta_t_milli](){
        t_ = t_ + delta_t_milli/1000.;
        double w = 1.2;
        double r = 0.05;
        auto temp = sign_*r*sin(w*t_);
        if(temp<0.){
          foot_count_++;
          if(foot_count_ == dof_) {
            foot_count_=0;
          }
          sign_ = -sign_;
          temp = -temp;
        }
        for(int i=0; i<dof_;i++) {
          limb_msgs::msg::Pxyz xyz_msg;
          xyz_msg.x = 0.2;
          xyz_msg.y = 0.0;
          xyz_msg.z = -0.05;
          if(i==foot_count_) {
            xyz_msg.z = xyz_msg.z + temp;
          }
          xyz_publishers_[i]->publish(xyz_msg);
        }
      }
    );
  }

  /*
  void TestGait::declare_parameters(){
    // robot geometry
    node_->declare_parameter<int>("modified_dh.dof");
    node_->declare_parameter<std::vector<double>>("modified_dh.a", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.d", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.alfa", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.eef_trans", std::vector<double>());
    node_->declare_parameter<std::vector<double>>("modified_dh.q0", std::vector<double>());
  }

  
  void TestGait::load_parameter() {
    int dof;
    if(node_->get_parameter("modified_dh.dof", dof)) {
      RCLCPP_INFO(node_->get_logger(), "limb dof loaded successfully, with value: ", std::to_str(dof).c_str());  
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
     
    for(int i = 0; i < dof; i++) {
      std::string temp = "Joint_" + std::to_string(i);
      joints_names.push_back(temp);
    }
  }*/

}  // penta_pod::kin::test_gait