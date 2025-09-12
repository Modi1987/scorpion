#ifndef PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_
#define PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_

#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>

namespace penta_pod::kin::gait_generator {

  enum class FootUpMotionType { CYCLOID, POLY3, SINUSOIDAL, UNKNOWN };

  inline FootUpMotionType foot_up_motion_type_from_string(const std::string& name) {
    if (name == "cycloid") return FootUpMotionType::CYCLOID;
    if (name == "poly3") return FootUpMotionType::POLY3;
    if (name == "sinusoidal") return FootUpMotionType::SINUSOIDAL;
    return FootUpMotionType::UNKNOWN;
  }

  inline std::string foot_up_motion_type_to_string(FootUpMotionType type) {
    switch (type) {
      case FootUpMotionType::CYCLOID: return "cycloid";
      case FootUpMotionType::POLY3: return "poly3";
      case FootUpMotionType::SINUSOIDAL: return "sinusoidal";
      default: return "unknown";
    }
  }

  inline auto double_to_string_formatted(double number, int digits_after_point)
    -> std::string {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(digits_after_point) << number;
    std::string formatted_string = oss.str();
    return formatted_string;
  };

  // move feet up (z up) calculation
  inline double foot_pos_z_generator(double b, double q, double phase_shift, int n, FootUpMotionType foot_up_motion_type) {
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
      switch (foot_up_motion_type) {
        case FootUpMotionType::CYCLOID:
          z = b * (1 - std::cos(2 * M_PI * u)) / 2;
          break;
        case FootUpMotionType::POLY3:
          z = b * (3 * u * u - 2 * u * u * u); // 3u^2 - 2u^3
          break;
        case FootUpMotionType::SINUSOIDAL:
          z = b * std::sin(M_PI * u);
          break;
        case FootUpMotionType::UNKNOWN:
        default:
          z = b * std::sin(M_PI * u);
          break;
      }
      return z;
    } else {
      return 0.0;
    }
  }

} // namespace penta_pod::kin::gait_generator

#endif // PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_
