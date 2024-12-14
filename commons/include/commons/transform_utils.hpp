#ifndef COMMONS_TRANS_UTILS_HPP_
#define COMMONS_TRANS_UTILS_HPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "rclcpp/rclcpp.hpp"
#include <array>
#include <cmath>
#include <numeric>
#include <optional>
#include <vector>

namespace penta_pod::kin::commons {

using geometry_msgs::msg::PoseStamped;
using geometry_msgs::msg::Transform;

auto get_translation_difference(const Transform &target,
                                const Transform &source)
    -> std::array<double, 3> {
  return {target.translation.x - source.translation.x,
          target.translation.y - source.translation.y,
          target.translation.z - source.translation.z};
}

auto get_position_difference(const PoseStamped &target,
                             const PoseStamped &source)
    -> std::array<double, 3> {
  return {target.pose.position.x - source.pose.position.x,
          target.pose.position.y - source.pose.position.y,
          target.pose.position.z - source.pose.position.z};
}

auto get_norm_from_vec(const std::vector<double> &vec) -> double {
  double sum = 0.0;
  for (auto v : vec) {
    sum += v * v;
  }
  return std::sqrt(sum);
}

auto get_direction_from_vec(const std::vector<double> &vec, double norm)
    -> std::optional<std::vector<double>> {
  if (norm <= 0.) {
    return std::nullopt;
  }
  std::vector<double> output_vec;
  output_vec.reserve(vec.size());
  for (const auto &v : vec) {
    output_vec.push_back(v / norm);
  }
  return output_vec;
}

auto interpolate_transform(const rclcpp::Node::SharedPtr /*node*/,
                           Transform target, Transform source,
                           double linear_vel, double /*angular_vel*/,
                           double dt_sec) -> std::optional<Transform> {

  Transform interpolated{};

  auto e = get_translation_difference(target, source);
  std::vector<double> e_vec{e.begin(), e.end()};
  auto norm = get_norm_from_vec(e_vec);

  double linear_displacement = linear_vel * dt_sec;
  if (norm < linear_displacement) { // almost near eachothers
    /*
    RCLCPP_INFO(node->get_logger(),
                "norm %f is less than the discrete displacement %f ", norm,
                linear_displacement);
    */
    interpolated = target;
    return interpolated;
  }

  auto dir_optional = get_direction_from_vec(e_vec, norm);
  if (!dir_optional.has_value()) {
    return std::nullopt;
  }
  auto dir = dir_optional.value();

  interpolated.translation.x =
      source.translation.x + dir[0] * linear_displacement;
  interpolated.translation.y =
      source.translation.y + dir[1] * linear_displacement;
  interpolated.translation.z =
      source.translation.z + dir[2] * linear_displacement;

  return interpolated;
}

auto interpolate_pose(const rclcpp::Node::SharedPtr node, PoseStamped target,
                      PoseStamped source, double linear_vel,
                      double /*angular_vel*/, double dt_sec)
    -> std::optional<PoseStamped> {

  auto get_parent_frame_id = [](PoseStamped msg) -> std::string {
    constexpr auto parnet_frame_id = "base_footprint";
    auto frame_id = msg.header.frame_id;
    if (frame_id == "") {
      return parnet_frame_id;
    }
    return frame_id;
  };

  auto source_parent_frame = get_parent_frame_id(source);
  auto target_parent_frame = get_parent_frame_id(target);

  if (source_parent_frame != target_parent_frame) {
    RCLCPP_ERROR(
        node->get_logger(),
        "Frame ID mismatch: target.frame_id = '%s', source.frame_id = '%s'",
        target.header.frame_id.c_str(), source.header.frame_id.c_str());
    return std::nullopt;
  }

  PoseStamped interpolated{};
  interpolated.header.stamp = node->now();
  interpolated.header.frame_id = source.header.frame_id;

  auto e = get_position_difference(target, source);
  std::vector<double> e_vec{e.begin(), e.end()};
  auto norm = get_norm_from_vec(e_vec);

  double linear_displacement = linear_vel * dt_sec;
  if (norm < linear_displacement) {
    /*
    RCLCPP_INFO(node->get_logger(),
                "norm %f is less than the discrete displacement %f", norm,
                linear_displacement);
    */
    interpolated = target;
    return interpolated;
  }

  auto dir_optional = get_direction_from_vec(e_vec, norm);
  if (!dir_optional.has_value()) {
    return std::nullopt;
  }
  auto dir = dir_optional.value();

  interpolated.pose.position.x =
      source.pose.position.x + dir[0] * linear_displacement;
  interpolated.pose.position.y =
      source.pose.position.y + dir[1] * linear_displacement;
  interpolated.pose.position.z =
      source.pose.position.z + dir[2] * linear_displacement;

  interpolated.pose.orientation = source.pose.orientation;

  return interpolated;
}

} // namespace penta_pod::kin::commons

#endif // COMMONS_TRANS_UTILS_HPP_
