#include "penta_teleop/joystick_base_link_motion.hpp"
#include "commons/quaternion_utils.hpp"

namespace penta_pod::teleop::joystick_base_link_motion {

JoystickBaseLinkMotion::JoystickBaseLinkMotion(
    rclcpp::Node::SharedPtr node, rclcpp::Node::SharedPtr clients_node)
    : node_(node), clients_node_(clients_node) {
  // Initialize private members
  this->declare_parameters();
  this->get_parameters();
  initial_base_pose = nullptr;
  // Create  publishers/subscribers
  joy_subscriber_ = node_->create_subscription<JoyMsg>(
      "joy", 1, [this](const JoyMsg::SharedPtr msg) {
        this->joystick_msg_callback(msg);
      });
  // auto client_callback_group = node_->create_callback_group(
  //     rclcpp::CallbackGroupType::Reentrant);
  get_base_pose_ = clients_node_->create_client<GetCurrentBasePose>(
      base_link_motion_params_.get_base_pose_service_name,
      rclcpp::QoS(rclcpp::ServicesQoS()));
  set_base_pose_ = clients_node_->create_client<SetTargetBasePose>(
      base_link_motion_params_.set_base_pose_service_name,
      rclcpp::QoS(rclcpp::ServicesQoS()));
  // publisher
  set_base_pose_pub_ = node_->create_publisher<PoseStampedMsg>(
      base_link_motion_params_.set_base_pose_topic_name, 1);
}

void JoystickBaseLinkMotion::joystick_msg_callback(
    const JoyMsg::SharedPtr msg) {
  auto enable_button_value =
      msg->buttons[base_link_motion_params_.enable_button_index];
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

void JoystickBaseLinkMotion::joy_msg_to_base_link_motion(
    const JoyMsg::SharedPtr msg) {
  // Check if initial pose is stored
  if (initial_base_pose == nullptr) {
    auto message = "Initial pose is not stored! can not calculate target";
    RCLCPP_ERROR(node_->get_logger(), message);
    return;
  }
  // Check if fileterd value is stored
  if (filtered_base_pose == nullptr) {
    filtered_base_pose = std::make_shared<PoseStampedMsg>();
    filtered_base_pose->pose = initial_base_pose->pose;
    auto message = "filtered pose is not stored! can not calculate target";
    RCLCPP_ERROR(node_->get_logger(), message);
    return;
  }
  get_parameters();
  // Check if initial pose is in limits
  auto initial_z = initial_base_pose->pose.position.z;
  auto initial_z_limit = base_link_motion_params_.minimum_init_z_value;
  if (initial_z < initial_z_limit) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Initial pose z: {%f} less than limit {%f}", initial_z,
                 initial_z_limit);
    return;
  }
  double yaw_command = msg->axes[base_link_motion_params_.yaw_axis_index] *
                       base_link_motion_params_.yaw_scale;
  double pitch_command = msg->axes[base_link_motion_params_.pitch_axis_index] *
                         base_link_motion_params_.pitch_scale;
  double x_command = msg->axes[base_link_motion_params_.x_axis_index] *
                     base_link_motion_params_.x_scale;
  double y_command = msg->axes[base_link_motion_params_.y_axis_index] *
                     base_link_motion_params_.y_scale;
  // compensate with intial pose
  x_command += initial_base_pose->pose.position.x;
  y_command += initial_base_pose->pose.position.y;
  double z_command = initial_base_pose->pose.position.z;
  auto target_base_pose = std::make_shared<PoseStampedMsg>();
  target_base_pose->pose.position.x = x_command;
  target_base_pose->pose.position.y = y_command;
  target_base_pose->pose.position.z = z_command;
  using namespace penta_pod::kin::commons::quaternion_utils;
  target_base_pose->pose.orientation =
      rpy_to_quaternion(yaw_command, pitch_command, 0.0);
  // Filter data
  auto c = base_link_motion_params_.filter_value;
  auto filter = [](double output, double input, double filter) -> double {
    return output * (1.0 - filter) + input * filter;
  };
  filtered_base_pose->pose.position.x =
      filter(filtered_base_pose->pose.position.x,
             target_base_pose->pose.position.x, c);
  filtered_base_pose->pose.position.y =
      filter(filtered_base_pose->pose.position.y,
             target_base_pose->pose.position.y, c);
  filtered_base_pose->pose.position.z =
      filter(filtered_base_pose->pose.position.z,
             target_base_pose->pose.position.z, c);
  filtered_base_pose->pose.orientation = target_base_pose->pose.orientation;
  set_target_base_pose(filtered_base_pose);
  RCLCPP_INFO_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 1000,
      "Base Link Motion Commands - X: %f, Y: %f, Yaw: %f, Pitch: %f", x_command,
      y_command, yaw_command, pitch_command);
}

void JoystickBaseLinkMotion::store_current_base_pose() {
  // check if service exists
  if (!get_base_pose_->wait_for_service(std::chrono::seconds(100))) {
    RCLCPP_ERROR(node_->get_logger(), "Service %s is unavailable.",
                 base_link_motion_params_.get_base_pose_service_name.c_str());
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

void JoystickBaseLinkMotion::set_target_base_pose(
    const PoseStampedMsg::SharedPtr target_base_pose) {
  // If command by topic (command is set with no interpolation on server)
  if (base_link_motion_params_.command_by_topic) {
    set_base_pose_pub_->publish(*target_base_pose);
    return;
  }
  // If command by service, command is ineterpolated on server side
  // check if service exists
  if (!set_base_pose_->wait_for_service(std::chrono::seconds(100))) {
    RCLCPP_ERROR(node_->get_logger(), "Service %s is unavailable.",
                 base_link_motion_params_.set_base_pose_service_name.c_str());
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

void JoystickBaseLinkMotion::restore_initial_base_pose() {
  set_target_base_pose(initial_base_pose);
  filtered_base_pose = nullptr;
}

void JoystickBaseLinkMotion::declare_parameters() {
  // Declare parameters if needed
  node_->declare_parameter<int>("enable_base_motion_button_index", 4);
  node_->declare_parameter<int>("base_link_yaw_axis_index", 3);
  node_->declare_parameter<int>("base_link_pitch_axis_index", 4);
  node_->declare_parameter<int>("base_link_x_axis_index", 1);
  node_->declare_parameter<int>("base_link_y_axis_index", 0);
  node_->declare_parameter<double>("base_link_yaw_scale", 0.0);
  node_->declare_parameter<double>("base_link_pitch_scale", 0.0);
  node_->declare_parameter<double>("base_link_x_scale", 0.0);
  node_->declare_parameter<double>("base_link_y_scale", 0.0);
  node_->declare_parameter<bool>("command_by_topic", true);
  node_->declare_parameter<std::string>("service_name.get_base_pose",
                                        "base_twerk/get_current_null_pose");
  node_->declare_parameter<std::string>("service_name.set_base_pose",
                                        "base_twerk/cmd_null_setpoint");
  node_->declare_parameter<std::string>("topic_name.set_base_pose",
                                        "set_base_pose");
  node_->declare_parameter<double>("limits.minimum_init_z_value", 0.1);
}

void JoystickBaseLinkMotion::get_parameters() {
  node_->get_parameter("enable_base_motion_button_index",
                       base_link_motion_params_.enable_button_index);
  node_->get_parameter("base_link_yaw_axis_index",
                       base_link_motion_params_.yaw_axis_index);
  node_->get_parameter("base_link_pitch_axis_index",
                       base_link_motion_params_.pitch_axis_index);
  node_->get_parameter("base_link_x_axis_index",
                       base_link_motion_params_.x_axis_index);
  node_->get_parameter("base_link_y_axis_index",
                       base_link_motion_params_.y_axis_index);
  node_->get_parameter("base_link_yaw_scale",
                       base_link_motion_params_.yaw_scale);
  node_->get_parameter("base_link_pitch_scale",
                       base_link_motion_params_.pitch_scale);
  node_->get_parameter("base_link_x_scale", base_link_motion_params_.x_scale);
  node_->get_parameter("base_link_y_scale", base_link_motion_params_.y_scale);
  node_->get_parameter("command_by_topic",
                       base_link_motion_params_.command_by_topic);
  node_->get_parameter("service_name.get_base_pose",
                       base_link_motion_params_.get_base_pose_service_name);
  node_->get_parameter("service_name.set_base_pose",
                       base_link_motion_params_.set_base_pose_service_name);
  node_->get_parameter("topic_name.set_base_pose",
                       base_link_motion_params_.set_base_pose_topic_name);
  node_->get_parameter("limits.minimum_init_z_value",
                       base_link_motion_params_.minimum_init_z_value);
}

} // namespace penta_pod::teleop::joystick_base_link_motion