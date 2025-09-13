#ifndef PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_
#define PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <unordered_map>

namespace penta_pod::kin::gait_generator {

inline auto double_to_string_formatted(double number, int digits_after_point)
    -> std::string {
  std::ostringstream oss;
  oss << std::fixed << std::setprecision(digits_after_point) << number;
  std::string formatted_string = oss.str();
  return formatted_string;
}

class FootUpMotion {

public:
  enum class Type { CYCLOID, POLY3, SINUSOIDAL, UNKNOWN };

private:
  std::unordered_map<Type, std::string> foot_up_motion_type_map;

  Type foot_up_motion_type_{Type::UNKNOWN};

public:
  FootUpMotion() {
    foot_up_motion_type_map = {{Type::CYCLOID, "cycloid"},
                               {Type::POLY3, "poly3"},
                               {Type::SINUSOIDAL, "sinusoidal"},
                               {Type::UNKNOWN, "unknown"}};
  }

  void set_foot_up_motion_type(Type type) { foot_up_motion_type_ = type; }

  Type foot_up_motion_type_from_string(const std::string &name) {
    for (const auto &pair : foot_up_motion_type_map) {
      if (pair.second == name) {
        return pair.first;
      }
    }
    return Type::UNKNOWN;
  }

  std::string foot_up_motion_type_to_string(Type type) {
    for (const auto &pair : foot_up_motion_type_map) {
      if (pair.first == type) {
        return pair.second;
      }
    }
    return foot_up_motion_type_map[Type::UNKNOWN];
  }

  // move feet up (z up) calculation
  double foot_pos_z_generator(double b, double q, double phase_shift, int n) {
    q = q + phase_shift; // add the phase
    q = q - std::floor(q / (2 * M_PI)) * 2 *
                M_PI; // remove multiples of 2*pi (resulting q is always less
                      // than 2*pi)
    double epsilon = M_PI / n;

    if (q < ((n - 1) * 2 * epsilon)) { // interval where feet is on ground
      return 0.0;
    } else if (q < 2 * M_PI) { // when feet is off the ground
      double u = (q - (n - 1) * 2 * epsilon) / (2 * epsilon); // u in [0, 1]
      double z = 0.0;
      switch (foot_up_motion_type_) {
      case Type::CYCLOID:
        z = b * (1 - std::cos(2 * M_PI * u)) / 2;
        break;
      case Type::POLY3:
        z = b * (3 * u * u - 2 * u * u * u); // 3u^2 - 2u^3
        break;
      case Type::SINUSOIDAL:
        z = b * std::sin(M_PI * u);
        break;
      case Type::UNKNOWN:
      default:
        z = b * std::sin(M_PI * u);
        break;
      }
      return z;
    } else {
      return 0.0;
    }
  }
};

} // namespace penta_pod::kin::gait_generator

#endif // PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_
