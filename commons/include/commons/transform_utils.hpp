#ifndef COMMONS_TRANS_UTILS_HPP_
#define COMMONS_TRANS_UTILS_HPP_

#include <optional>
#include <cmath>
#include <array>
#include <vector>
#include <numeric>
#include "geometry_msgs/msg/transform.hpp"

namespace penta_pod::kin::commons {

  using Transform = geometry_msgs::msg::Transform;

  auto get_translation_difference(const Transform& target, const Transform& source) -> std::array<double, 3> {
    return {target.translation.x - source.translation.x,
            target.translation.y - source.translation.y,
            target.translation.z - source.translation.z};
  }

  auto get_norm_from_vec(const std::vector<double>& vec) -> double {
    return std::sqrt(std::accumulate(vec.begin(), vec.end(), 0.0, 
                                     [](double sum, double v) { return sum + v * v; }));
  }

  auto get_direction_from_vec(const std::vector<double>& vec, double norm) -> std::optional<std::vector<double>> {
    if (norm <= 0.) {
      return std::nullopt;
    }
    std::vector<double> output_vec;
    output_vec.reserve(vec.size());
    for (const auto& v : vec) {
      output_vec.push_back(v / norm);
    }
    return output_vec;
  }

  auto interpolate_transform(Transform target, Transform source, 
                              double linear_vel, double /*angular_vel*/,
                              double dt_sec) -> std::optional<Transform> {

    Transform interpolated {};

    auto e = get_translation_difference(target, source);
    std::vector<double> e_vec{e.begin(), e.end()};
    auto norm = get_norm_from_vec(e_vec);

    double linear_displacement = linear_vel * dt_sec;
    if (norm < linear_displacement) { // almost near eachothers
      interpolated = source;
      return interpolated;
    }

    auto dir_optional = get_direction_from_vec(e_vec, norm);
    if (!dir_optional.has_value()) {
      return std::nullopt;
    }
    auto dir = dir_optional.value();

    interpolated.translation.x += dir[0] * linear_displacement;
    interpolated.translation.y += dir[1] * linear_displacement;
    interpolated.translation.z += dir[2] * linear_displacement;

    return interpolated;
  }

} // namespace penta_pod::kin::commons

#endif  // COMMONS_TRANS_UTILS_HPP_
