#include "penta_teleop/joystick_extra_controls.hpp"
#include "commons/quaternion_utils.hpp"

namespace penta_pod::teleop::joystick_extra_controls {

JoystickExtraControls::JoystickExtraControls()
    : node_{rclcpp::Node::make_shared("joystick_extra_controls")} {
  RCLCPP_INFO(node_->get_logger(),
              "Starting joystick_extra_controls, subscribing to joy topic and "
              "calling base_pose_setpoint service");
  // load parameters
  this->declare_parameters();
  this->get_parameters();

  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

  // Joystick subscriber
  joy_subscriber_ = node_->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, [this](const sensor_msgs::msg::Joy::SharedPtr msg) {
        this->joy_sub_callback(msg);
      });

  // Create service clients
  get_current_base_pose_client_ = node_->create_client<GetCurrentBasePose>(
      "get_current_null_pose", rmw_qos_profile_services_default,
      callback_group_);
  set_base_pose_client_ = node_->create_client<BasePoseSetpoint>(
      "cmd_null_setpoint", rmw_qos_profile_services_default, callback_group_);
  // Service cleint to change walking pattern
  set_gait_pattern_client_ = node_->create_client<SetGaitPattern>(
      "gait_generator/set_gait_pattern", rmw_qos_profile_services_default,
      callback_group_);
  
  // Action client to call base twerk action
  base_twerk_action_client_ =
      rclcpp_action::create_client<BaseTwerkAction>(node_, "base_twerk_action",
                                                    callback_group_);
}

void JoystickExtraControls::joy_sub_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {

  // move base up and down or yaw control
  dpad_up_down(msg);
  // change walking pattern
  set_gait_pattern(msg);
  // change "twerk it!" mode
  check_twerk_it_mode_switch(msg);
  // check "twerk it!" button
  check_twerk_pressed(msg);
}

void JoystickExtraControls::check_twerk_it_mode_switch(
  const sensor_msgs::msg::Joy::SharedPtr msg) {
  static int last_button_state = 0;
  auto index = twerk_it_params_.get_twerk_it_switch_mode_button_index();
  auto received_button_value = msg->buttons[index];

  if (received_button_value - last_button_state == 1) {
    // Button was pressed
    RCLCPP_INFO(node_->get_logger(),
                "Button to switch shaking mode pressed, changing mode");
    twerk_it_params_.switch_twerk_it_mode();

    RCLCPP_INFO(node_->get_logger(),
                "Applied twerk mode: %s, magnitude: %f",
                twerk_it_params_.get_current_twerk_it_mode_name().c_str(),
                twerk_it_params_.get_current_twerk_it_magnitude());
  }
  last_button_state = received_button_value;
}

void JoystickExtraControls::check_twerk_pressed(
  const sensor_msgs::msg::Joy::SharedPtr msg) {

  static int last_button_state = 0;
  auto index = twerk_it_params_.get_twerk_it_trigger_button_index();
  auto received_button_value = msg->buttons[index];
  
  if (received_button_value - last_button_state  == 1) {

    if (!is_null_space_motion_possible_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Base motion is not available, skipping base pose change.");
      return;
    }

    auto twerk_axis_index = twerk_it_params_.get_current_twerk_it_axis();
    auto magnitude = twerk_it_params_.get_current_twerk_it_magnitude();
    auto twerk_time_millis =
        twerk_it_params_.get_current_twerk_it_dance_time_millis();
    
    this->send_base_twerk_goal(twerk_axis_index, magnitude, twerk_time_millis);
    
    return;
  }

  last_button_state = received_button_value;
}

void JoystickExtraControls::set_gait_pattern(
    const sensor_msgs::msg::Joy::SharedPtr msg) {

  static int last_button_state = 0;
  static int walking_pattern = 0;
  int current_button_state =
      msg->buttons[this->gait_patterns_ctl_button_index_];
  if (current_button_state == 1 && last_button_state == 0) {
    walking_pattern++;
    if (walking_pattern > num_of_gait_patterns_) {
      walking_pattern = 0;
    }
    // Button was pressed
    RCLCPP_INFO(node_->get_logger(),
                "Button index %d pressed, changing walking pattern",
                this->gait_patterns_ctl_button_index_);
    // Check if the service is available before calling
    if (!set_gait_pattern_client_->wait_for_service(
            std::chrono::seconds(100))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Service set_gait_pattern_client_ is unavailable.");
      return;
    }
    // Add your logic to change the walking pattern here
    auto request = std::make_shared<SetGaitPattern::Request>();
    request->pattern = walking_pattern;
    // Call the service asynchronously
    auto future = set_gait_pattern_client_->async_send_request(
        request, [this](rclcpp::Client<SetGaitPattern>::SharedFuture response) {
          auto result = response.get();

          if (!result) {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed change walking pattern (null return).");
            return;
          }
          if (result->success) {
            RCLCPP_INFO(node_->get_logger(),
                        "Walking pattern changed successfully.");
          } else {
            RCLCPP_ERROR(node_->get_logger(),
                         "Failed to change walking pattern.");
          }
        });
  }
  last_button_state = current_button_state;
}

void JoystickExtraControls::dpad_up_down(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  if (msg->axes[this->d_pad_up_down_axis_index_] != 0) {
    
    if (!is_null_space_motion_possible_) {
      RCLCPP_WARN(node_->get_logger(),
                  "Base motion is not available, skipping base pose change.");
      return;
    }

    double delta_z = 0.001;
    double z_disp_base_command = msg->axes[7] * delta_z;

    double delta_pitch = 0.01;
    double pitch_disp_base_command = msg->axes[7] * delta_pitch;

    if (msg->buttons[5] == 1) {
      // If button 5 is pressed, use yaw control instead of z control
      z_disp_base_command = 0.0;
    } else {
      pitch_disp_base_command = 0.0;
    }

    // Check if the service is available before calling
    if (!get_current_base_pose_client_->wait_for_service(
            std::chrono::seconds(100))) {
      RCLCPP_ERROR(node_->get_logger(),
                   "Service get_current_base_pose is unavailable.");
      return;
    }

    // Prepare the request
    auto request = std::make_shared<GetCurrentBasePose::Request>();

    // Call the service asynchronously with a callback
    auto future = get_current_base_pose_client_->async_send_request(
        request,
        [this, z_disp_base_command, pitch_disp_base_command](
            rclcpp::Client<GetCurrentBasePose>::SharedFuture response) {
          this->handle_get_base_pose_response(response, z_disp_base_command,
                                              pitch_disp_base_command);
        });
  }
}

void JoystickExtraControls::handle_get_base_pose_response(
    rclcpp::Client<GetCurrentBasePose>::SharedFuture response,
    double z_disp_base_command, double pitch_disp_base_command) {

  auto result = response.get();

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to get base pose.");
    return;
  }

  // Sanity checks
  if ((result->pose.pose.position.z > this->z_base_max_value_) &&
      z_disp_base_command > 0.) {
    RCLCPP_ERROR(node_->get_logger(),
                 "Already above the max Z limit, can not move up any further.");
    return;
  }

  if ((result->pose.pose.position.z < this->z_base_min_value_) &&
      z_disp_base_command < 0.) {
    RCLCPP_ERROR(
        node_->get_logger(),
        "Already below the min Z limit, can not move down any further.");
    return;
  }

  RCLCPP_INFO(node_->get_logger(), "Current base pose: %f %f %f %f %f %f %f",
              result->pose.pose.position.x, result->pose.pose.position.y,
              result->pose.pose.position.z, result->pose.pose.orientation.x,
              result->pose.pose.orientation.y, result->pose.pose.orientation.z,
              result->pose.pose.orientation.w);

  // Prepare new base pose setpoint request
  auto setpoint_request = std::make_shared<BasePoseSetpoint::Request>();
  setpoint_request->pose = result->pose;
  setpoint_request->pose.pose.position.z += z_disp_base_command;

  using namespace penta_pod::kin::commons::quaternion_utils;
  auto pitch_quaternion = rpy_to_quaternion(0.0, pitch_disp_base_command, 0.0); // yaw, pitch, roll
  setpoint_request->pose.pose.orientation =
      hamilton_product(setpoint_request->pose.pose.orientation, pitch_quaternion);
  
  // Call the set_base_pose service asynchronously
  auto future = set_base_pose_client_->async_send_request(
      setpoint_request,
      [this](rclcpp::Client<BasePoseSetpoint>::SharedFuture setpoint_response) {
        this->handle_set_base_pose_response(setpoint_response);
      });
}

void JoystickExtraControls::handle_set_base_pose_response(
    rclcpp::Client<BasePoseSetpoint>::SharedFuture response) {

  auto result = response.get();

  if (!result) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to set base pose.");
  } else {
    RCLCPP_INFO(node_->get_logger(), "Base pose set successfully.");
  }
}

void JoystickExtraControls::send_base_twerk_goal(int twerk_axis_index, double twerk_magnitude, int twerk_time_millis) {
  
  if (!this->base_twerk_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(), "Action server not available after waiting");
    return;
  }

  auto goal_msg = BaseTwerkAction::Goal();

  goal_msg.r[twerk_axis_index] = twerk_magnitude; // in meters or rads, depending on axis
  goal_msg.w = twerk_it_params_.w; // rad/s
  goal_msg.dance_time_millis = twerk_time_millis; // milliseconds

  RCLCPP_INFO(node_->get_logger(), "Sending goal");

  auto send_goal_options = rclcpp_action::Client<BaseTwerkAction>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const rclcpp_action::ClientGoalHandle<BaseTwerkAction>::SharedPtr goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
      } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
        is_null_space_motion_possible_ = false;
      }
    };

  send_goal_options.feedback_callback =
    [this](const rclcpp_action::ClientGoalHandle<BaseTwerkAction>::SharedPtr,
            const std::shared_ptr<const BaseTwerkAction::Feedback> /*feedback*/) {
      RCLCPP_INFO(node_->get_logger(), "Received feedback");
    };
    
  send_goal_options.result_callback =
    [this](const rclcpp_action::ClientGoalHandle<BaseTwerkAction>::WrappedResult & result) {
      is_null_space_motion_possible_ = true;
      if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
      } else {
        RCLCPP_ERROR(node_->get_logger(), "Goal failed");
      }
    };

  this->base_twerk_action_client_->async_send_goal(goal_msg, send_goal_options);
}

} // namespace penta_pod::teleop::joystick_extra_controls
