#include "commons/ros2_utils.hpp" // here for check_intra_process_arg
#include "penta_teleop/joystick_base_link_motion.hpp"
#include "penta_teleop/joystick_turn_head_using_bumper.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  using penta_pod::kin::commons::is_intra_process_arg;
  bool use_intra = is_intra_process_arg(argc, argv);

  /* Base link motion x,y,z, pitch and yaw */
  auto base_link_motion_node_name = "joystick_base_link_motion_control";
  auto base_link_motion_clients_node_name = "joystick_base_link_motion_control_clients";
  rclcpp::Node::SharedPtr base_link_motion_node;
  rclcpp::Node::SharedPtr base_link_motion_clients_node;
  if (use_intra) {
    auto node_options =
      rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    base_link_motion_node = std::make_shared<rclcpp::Node>(base_link_motion_node_name, node_options);
    base_link_motion_clients_node = std::make_shared<rclcpp::Node>(base_link_motion_clients_node_name, node_options);
  } else {
    base_link_motion_node = std::make_shared<rclcpp::Node>(base_link_motion_node_name);
    base_link_motion_clients_node = std::make_shared<rclcpp::Node>(base_link_motion_clients_node_name);
  }

  // Create a shared node instance
  using namespace penta_pod::teleop::joystick_base_link_motion;
  auto joystick_base_link_motion = JoystickBaseLinkMotion(base_link_motion_node, base_link_motion_clients_node);

  /* Turn head using yaw */
  auto turn_head_motion_node_name = "turn_head_motion_control";
  auto turn_head_motion_clients_node_name = "turn_head_motion_control_clients";
  rclcpp::Node::SharedPtr turn_head_motion_node;
  rclcpp::Node::SharedPtr turn_head_motion_clients_node;
  if (use_intra) {
    auto node_options =
      rclcpp::NodeOptions().use_intra_process_comms(use_intra);
    turn_head_motion_node = std::make_shared<rclcpp::Node>(turn_head_motion_node_name, node_options);
    turn_head_motion_clients_node = std::make_shared<rclcpp::Node>(turn_head_motion_clients_node_name, node_options);
  } else {
    turn_head_motion_node = std::make_shared<rclcpp::Node>(turn_head_motion_node_name);
    turn_head_motion_clients_node = std::make_shared<rclcpp::Node>(turn_head_motion_clients_node_name);
  }

  // Create a shared node instance
  using namespace penta_pod::teleop::joystick_turn_head_using_bumpers;
  auto joystick_turn_head_motion = JoystickTurnHead(turn_head_motion_node, turn_head_motion_clients_node);

  // Multi-threaded executor
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(base_link_motion_node);
  executor.add_node(base_link_motion_clients_node);
  executor.add_node(turn_head_motion_node);
  executor.add_node(turn_head_motion_clients_node);

  // Spin the executor
  executor.spin();

  // Cleanup
  rclcpp::shutdown();
  return 0;
}
