// include librarries
#include "gait_generator/gait_generator.hpp"
#include <stdexcept>

namespace penta_pod::kin::gait_generator {

void GaitGenerator::declare_parameters() {
  node_->declare_parameter<int>("limbs_num");
  node_->declare_parameter<std::vector<double>>("legs_body_transforms",
                                                std::vector<double>{});
  node_->declare_parameter<std::vector<double>>(
      "init_feet_pos_in_basefootprint", std::vector<double>{});
  node_->declare_parameter<std::vector<double>>(
      "init_body_basefootprint_transform", std::vector<double>{});
  node_->declare_parameter<double>("gait_parameters.max_gait_linear_speed");
  node_->declare_parameter<double>("gait_parameters.max_gait_turning_speed");
  node_->declare_parameter<int>("gait_parameters.gait_patterns.num");
  node_->declare_parameter<std::vector<long int>>(
      "gait_parameters.gait_patterns.feet_order", std::vector<long int>{});
  node_->declare_parameter<double>("gait_parameters.gait_radial_frequency");
  node_->declare_parameter<double>("gait_parameters.step_height");
}

void GaitGenerator::load_parameters() {
  // Helper function to load a parameter and throw error if not found
  auto load_param = [this](const std::string &param_name, auto &param_value,
                           const std::string &error_message) {
    if (!node_->get_parameter(param_name, param_value)) {
      RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
      throw std::runtime_error(error_message);
    }
  };

  // Helper function to validate vector size
  auto validate_vector_size =
      [this]<typename T>(const std::vector<T> &vec, int expected_size,
                         const std::string &error_message) {
        if (static_cast<int>(vec.size()) != expected_size) {
          RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
          throw std::runtime_error(error_message);
        }
      };

  // Helper function to create a transform from values
  auto array_to_transform =
      [this](const std::vector<double> &transforms_vec,
             int frame_index) -> geometry_msgs::msg::Transform {
    auto minimal_required_size = 7 * frame_index + 7;
    if (transforms_vec.size() < static_cast<size_t>(minimal_required_size)) {
      auto error_message =
          "Error, can not create trasform from array at index " +
          std::to_string(frame_index) +
          " since minimal required size is less than " +
          std::to_string(minimal_required_size);
      RCLCPP_ERROR(node_->get_logger(), error_message.c_str());
      throw std::runtime_error(error_message);
    }
    geometry_msgs::msg::Transform transform;
    auto start_index = frame_index * 7;
    transform.translation.x = transforms_vec[start_index + 0];
    transform.translation.y = transforms_vec[start_index + 1];
    transform.translation.z = transforms_vec[start_index + 2];
    transform.rotation.x = transforms_vec[start_index + 3];
    transform.rotation.y = transforms_vec[start_index + 4];
    transform.rotation.z = transforms_vec[start_index + 5];
    transform.rotation.w = transforms_vec[start_index + 6];
    return transform;
  };

  // Load limbs number
  load_param("limbs_num", feet_num_, "No limbs_num parameter found.");
  RCLCPP_INFO(node_->get_logger(), "Loaded limbs_num value is: %d", feet_num_);

  // Load and validate leg-body transforms
  std::vector<double> transform_params;
  load_param("legs_body_transforms", transform_params,
             "No transform parameters found.");
  validate_vector_size(transform_params, feet_num_ * 7,
                       "Invalid transform parameter size, expected " +
                           std::to_string(feet_num_) + "x7 elements.");

  for (int frame_index = 0; frame_index < feet_num_; ++frame_index) {
    auto transform = array_to_transform(transform_params, frame_index);
    legs_body_transforms_.emplace_back(transform);

    std::string message = "Shoulder [" + std::to_string(frame_index) +
                          "] transform in body frame is: [ ";
    for (int i = 0; i < 7; ++i) {
      message +=
          double_to_string_formatted(transform_params[i + 7 * frame_index], 4) +
          " ";
    }
    message += "]";
    RCLCPP_INFO(node_->get_logger(), message.c_str());
  }

  // Load and validate initial feet positions
  std::vector<double> init_feet_pos_params;
  load_param("init_feet_pos_in_basefootprint", init_feet_pos_params,
             "No feet position parameters found.");
  validate_vector_size(init_feet_pos_params, feet_num_ * 3,
                       "Invalid feet position parameter size, expected " +
                           std::to_string(feet_num_) + "x3 elements.");

  for (int count = 0; count < feet_num_; ++count) {
    geometry_msgs::msg::Point xyz;
    xyz.x = init_feet_pos_params[0 + count * 3];
    xyz.y = init_feet_pos_params[1 + count * 3];
    xyz.z = init_feet_pos_params[2 + count * 3];
    feet_pos_in_footprint_.emplace_back(xyz);
    init_feet_pos_in_footprint_.emplace_back(xyz);
  }

  // Load and validate initial body-to-basefootprint transform
  std::vector<double> body_basefootprint_params;
  load_param("init_body_basefootprint_transform", body_basefootprint_params,
             "No init transform body to basefootprint found.");
  validate_vector_size(body_basefootprint_params, 7,
                       "Size of init_body_basefootprint_transform must be 7 (3 "
                       "for position followed by 4 quaternion).");

  auto transform = array_to_transform(body_basefootprint_params, 0);
  body_basefootprint_ = transform;

  // Load gait parameters
  load_param("gait_parameters.max_gait_linear_speed",
             gait_parameters_.max_gait_linear_speed,
             "Parameter gait_parameters.max_gait_linear_speed was not found.");
  RCLCPP_INFO(
      node_->get_logger(),
      "Loaded gait_parameters.max_gait_linear_speed value is: %f [m/sec]",
      gait_parameters_.max_gait_linear_speed);

  load_param("gait_parameters.max_gait_turning_speed",
             gait_parameters_.max_gait_turning_speed,
             "Parameter gait_parameters.max_gait_turning_speed was not found.");
  RCLCPP_INFO(
      node_->get_logger(),
      "Loaded gait_parameters.max_gait_turning_speed value is: %f [rad/sec]",
      gait_parameters_.max_gait_turning_speed);

  load_param("gait_parameters.gait_radial_frequency",
             gait_parameters_.gait_radial_frequency,
             "Parameter gait_parameters.gait_radial_frequency was not found.");
  RCLCPP_INFO(node_->get_logger(),
              "Loaded gait_parameters.gait_radial_frequency value is: %f [Hz]",
              gait_parameters_.gait_radial_frequency);

  load_param("gait_parameters.step_height", gait_parameters_.step_height,
             "Parameter gait_parameters.step_height was not found.");
  RCLCPP_INFO(node_->get_logger(),
              "Loaded gait_parameters.step_height value is: %f [m]",
              gait_parameters_.step_height);

  // load gait patterns
  load_param("gait_parameters.gait_patterns.num", gait_patterns_.gaits_num_,
             "Parameter gait_parameters.gait_patterns.num was not found.");
  RCLCPP_INFO(node_->get_logger(),
              "Loaded gait_parameters.gait_patterns value is: %d",
              gait_patterns_.gaits_num_);

  std::vector<long int> vector_gait_patterns;
  load_param("gait_parameters.gait_patterns.feet_order", vector_gait_patterns,
             "No init gait_parameters.gait_patterns.feet_order found.");
  int assert_vector_size = feet_num_ * gait_patterns_.gaits_num_;
  std::string if_error_message =
      "Size of gait_parameters.gait_patterns.feet_order must be " +
      std::to_string(assert_vector_size);
  validate_vector_size(vector_gait_patterns, assert_vector_size,
                       if_error_message);
  // Populate the patterns.
  int count = 0;
  gait_patterns_.patterns_.resize(
      gait_patterns_.gaits_num_); // resize to number of gaits
  for (int i = 0; i < gait_patterns_.gaits_num_; i++) {
    gait_patterns_.patterns_[i].resize(feet_num_); // resize to fit the feet
    std::string log_message =
        "Loaded gait_pattern [" + std::to_string(i) + "]: ";
    for (int j = 0; j < feet_num_; j++) {
      int temp = vector_gait_patterns[count];
      if (temp < 0) {
        std::string error_message =
            "Foot index at gait_parameters.gait_patterns.feet_order[" +
            std::to_string(count) + "] shall not be less than zero";
        throw std::runtime_error(error_message);
      }
      if (temp >= feet_num_) {
        std::string error_message =
            "Foot index at gait_parameters.gait_patterns.feet_order[" +
            std::to_string(count) +
            "] shall not be more nor equal to the number of feet, specified "
            "as " +
            std::to_string(feet_num_);
        throw std::runtime_error(error_message);
      }
      int foot_index = vector_gait_patterns[count];
      log_message = log_message + std::to_string(foot_index) + " |";
      gait_patterns_.patterns_[i][j] = foot_index;
      count++;
    }
    RCLCPP_INFO(node_->get_logger(), log_message.c_str());
  }
}

} // namespace penta_pod::kin::gait_generator
