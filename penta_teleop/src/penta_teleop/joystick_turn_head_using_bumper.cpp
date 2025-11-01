#include "penta_teleop/joystick_turn_head_using_bumper.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace penta_pod::teleop::joystick_turn_head_using_bumpers {

JoystickTurnHead::JoystickTurnHead(
    rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr clients_node)
    : node_(node), clients_node_(clients_node) {
  // Initialize private members
  this->declare_parameters();
  this->get_parameters();
  initial_base_pose = nullptr;
  // Create  publishers/subscribers
  joy_subscriber_ = node_->create_subscription<JoyMsg>(
      "joy", 10, [this](const JoyMsg::SharedPtr msg) {
        this->joystick_msg_callback(msg);
      });
  // auto client_callback_group = node_->create_callback_group(
  //     rclcpp::CallbackGroupType::Reentrant);
  get_base_pose_ = clients_node_->create_client<GetCurrentBasePose>(
      turn_left_right_motion_params_.get_base_pose_service_name,
      rclcpp::QoS(rclcpp::ServicesQoS()));
  set_base_pose_ = clients_node_->create_client<SetTargetBasePose>(
      turn_left_right_motion_params_.set_base_pose_service_name,
      rclcpp::QoS(rclcpp::ServicesQoS()));
  // publisher
  set_base_pose_pub_ = node_->create_publisher<PoseStampedMsg>(
      turn_left_right_motion_params_.set_base_pose_topic_name, 1);
}

void JoystickTurnHead::joystick_msg_callback(
    const JoyMsg::SharedPtr msg) {
  auto enable_button_value =
      msg->buttons[turn_left_right_motion_params_.enable_button_index];
  if ((previous_enable_button_value == 0) && (enable_button_value == 1)) {
    // store initial base link pose
    RCLCPP_INFO(node_->get_logger(), "Storing current base pose!");
    previous_enable_button_value =
        enable_button_value; // very important, shall be before service call
    this->store_current_base_pose();
    return;
  }
  if ((previous_enable_button_value == 1) && (enable_button_value == 0)) {
    // setback initial base link pose
    RCLCPP_INFO(node_->get_logger(), "Restoring initial pose!");
    previous_enable_button_value =
        enable_button_value; // very important, shall be before service call
    this->restore_initial_base_pose();
    return;
  }
  if ((previous_enable_button_value == 1) && (enable_button_value == 1)) {
    if (!this->query_pose_ready) {
      previous_enable_button_value = enable_button_value;
      return;
    }
    joy_msg_to_base_link_motion(msg);
    return;
  }
  previous_enable_button_value = enable_button_value;
}

void JoystickTurnHead::joy_msg_to_base_link_motion(
    const JoyMsg::SharedPtr msg) {
  static rclcpp::Time last_time = node_->get_clock()->now();
  // Check if initial pose is stored
  if (initial_base_pose == nullptr) {
    auto message = "Initial pose is not stored! can not calculate target";
    RCLCPP_ERROR(node_->get_logger(), message);
    return;
  }

  get_parameters(); // to update parameter values if changing on the fly
  double factor = 0.;
  factor += (-1.0 + msg->axes[turn_left_right_motion_params_.yaw_turn_right_axis_index]) / 2.0;
  factor -= (-1.0 + msg->axes[turn_left_right_motion_params_.yaw_turn_left_axis_index]) / 2.0;
  double yaw_command = factor * turn_left_right_motion_params_.yaw_angle_scale;
  // filter command
  double c = turn_left_right_motion_params_.filter;
  filtered_yaw_cmd = c * yaw_command + (1.0 - c) * filtered_yaw_cmd;
  // compensate with intial pose
  double x_command = initial_base_pose->pose.position.x;
  double y_command = initial_base_pose->pose.position.y;
  double z_command = initial_base_pose->pose.position.z;
  auto target_base_pose = std::make_shared<PoseStampedMsg>();
  target_base_pose->pose.position.x = x_command;
  target_base_pose->pose.position.y = y_command;
  target_base_pose->pose.position.z = z_command;
  tf2::Quaternion q_initial, q_yaw;
  tf2::fromMsg(initial_base_pose->pose.orientation, q_initial);
  q_yaw.setRPY(0.0, 0.0, filtered_yaw_cmd);
  tf2::Quaternion q_command = q_yaw * q_initial;
  target_base_pose->pose.orientation = tf2::toMsg(q_command);

  rclcpp::Time current_time = node_->get_clock()->now();
  constexpr double update_interval = 0.1;
  if ((current_time - last_time) < rclcpp::Duration::from_seconds(update_interval)) {
    return;
  }
  last_time = current_time;

  set_target_base_pose(target_base_pose);
}

void JoystickTurnHead::store_current_base_pose() {
  // check if service exists
  if (!get_base_pose_->wait_for_service(std::chrono::seconds(100))) {
    RCLCPP_ERROR(node_->get_logger(), "Service %s is unavailable.",
                 turn_left_right_motion_params_.get_base_pose_service_name.c_str());
    return;
  }
  // call service
  auto request = std::make_shared<GetCurrentBasePose::Request>();
  this->query_pose_ready = false;
  auto future = get_base_pose_->async_send_request(
      request,
      [this](rclcpp::Client<GetCurrentBasePose>::SharedFuture response) {
        auto result = response.get();
        if (!result) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Failed to get base link pose (null return).");
          return;
        }
        auto current_base_pose = result->pose.pose;
        RCLCPP_INFO(node_->get_logger(),
                    "Current base pose (x: %f, y: %f, z: %f)",
                    current_base_pose.position.x, current_base_pose.position.y,
                    current_base_pose.position.z);
        this->initial_base_pose = std::make_shared<PoseStampedMsg>();
        this->initial_base_pose->pose.position = current_base_pose.position;
        this->initial_base_pose->pose.orientation =
            current_base_pose.orientation;
        this->query_pose_ready = true;
      });
}

void JoystickTurnHead::set_target_base_pose(
    const PoseStampedMsg::SharedPtr target_base_pose) {
  // If command by topic (command is set with no interpolation on server)
  if (turn_left_right_motion_params_.command_by_topic) {
    set_base_pose_pub_->publish(*target_base_pose);
    return;
  }
  // If command by service, command is ineterpolated on server side
  // check if service exists
  if (!set_base_pose_->wait_for_service(std::chrono::seconds(100))) {
    RCLCPP_ERROR(node_->get_logger(), "Service %s is unavailable.",
                 turn_left_right_motion_params_.set_base_pose_service_name.c_str());
    return;
  }
  // call service
  auto request = std::make_shared<SetTargetBasePose::Request>();
  request->pose = *target_base_pose;
  auto future = set_base_pose_->async_send_request(
      request,
      [this](rclcpp::Client<SetTargetBasePose>::SharedFuture response) {
        auto result = response.get();
        if (!result) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Failed to set base link pose (null return).");
          return;
        }
        if (!result->success) {
          RCLCPP_ERROR(node_->get_logger(),
                       "Failed to set base link pose (return failure).");
        }
      });
}

void JoystickTurnHead::restore_initial_base_pose() {
  set_target_base_pose(initial_base_pose);
}

void JoystickTurnHead::declare_parameters() {
  // Declare parameters if needed
  node_->declare_parameter<int>("turn_right_left_with_bumper.enable_base_motion_button_index", 4);
  node_->declare_parameter<int>("turn_right_left_with_bumper.yaw_turn_right_axis_index", 3);
  node_->declare_parameter<int>("turn_right_left_with_bumper.yaw_turn_left_axis_index", 3);
  node_->declare_parameter<double>("turn_right_left_with_bumper.yaw_angle_scale", 0.3);
  node_->declare_parameter<bool>("command_by_topic", true);
  node_->declare_parameter<std::string>("service_name.get_base_pose",
                                        "base_twerk/get_current_null_pose");
  node_->declare_parameter<std::string>("service_name.set_base_pose",
                                        "base_twerk/cmd_null_setpoint");
  node_->declare_parameter<std::string>("topic_name.set_base_pose",
                                        "set_base_pose");
  node_->declare_parameter<double>("turn_right_left_with_bumper.filter", 0.1);
}

void JoystickTurnHead::get_parameters() {
  node_->get_parameter("turn_right_left_with_bumper.enable_base_motion_button_index",
                       turn_left_right_motion_params_.enable_button_index);
  node_->get_parameter("turn_right_left_with_bumper.yaw_turn_right_axis_index",
                       turn_left_right_motion_params_.yaw_turn_right_axis_index);
  node_->get_parameter("turn_right_left_with_bumper.yaw_turn_left_axis_index",
                       turn_left_right_motion_params_.yaw_turn_left_axis_index);
  node_->get_parameter("turn_right_left_with_bumper.yaw_angle_scale",
                       turn_left_right_motion_params_.yaw_angle_scale);
  node_->get_parameter("command_by_topic", turn_left_right_motion_params_.command_by_topic);
  node_->get_parameter("service_name.get_base_pose",
                       turn_left_right_motion_params_.get_base_pose_service_name);
  node_->get_parameter("service_name.set_base_pose",
                       turn_left_right_motion_params_.set_base_pose_service_name);
  node_->get_parameter("topic_name.set_base_pose",
                       turn_left_right_motion_params_.set_base_pose_topic_name);
    node_->get_parameter("turn_right_left_with_bumper.filter",
                       turn_left_right_motion_params_.filter);
}

} // namespace penta_pod::teleop::joystick_turn_head_using_bumpers