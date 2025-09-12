#ifndef PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_
#define PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_

#include <cmath>
#include <string>
#include <sstream>
#include <iomanip>
#include "geometry_msgs/msg/quaternion.hpp"

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

  inline double simple_pos_interpolation(double alfa, double x0, double x1) {
    if (alfa < 0.0)
      return x0;
    else if (alfa > 1.0)
      return x1;
    else
      return (1.0 - alfa) * x0 + alfa * x1;
  }

  inline double pos_vel_interpolation_only_x1(
    double alfa, double alfa_dot, double x0, double x1, double v0, double v1) {
    // interpolation equation is:
    // x(alfa) = (c1*alfa + c2*alfa^2 + c3*alfa^3 ) * x1
    // dx(alfa)/dt = (c1 + 2*c2*alfa + 3*c3*alfa^2) * alfa_dot * x1
    // alfa in [0, 1]
    if (alfa < 0.0)
      return x0;
    else if (alfa > 1.0)
      return x1;
    else {
      if (std::abs(x1) < 1e-6) { // can not devide by zero
        return simple_pos_interpolation(alfa, x0, x1);
      }
      double c1 = (v0 / alfa_dot) / x1;
      double c2 = 3 -2*c1 - (v1 / alfa_dot) / x1;
      double c3 = 1 - c1 - c2;
      return (c1*alfa + c2*alfa*alfa + c3*alfa*alfa*alfa)*x1;
    }
  }

  inline double pos_vel_interpolation(
    double alfa, double alfa_dot, double x0, double x1, double v0, double v1) {
    // interpolation equation is:
    // x(alfa) = x0 * (1 - c1*alfa - c2*alfa^2 - c3*alfa^3 ) + alfa * x1
    // dx(alfa)/dt = x0 * (-c1*alfa_dot - 2*c2*alfa*alfa_dot - 3*c3*alfa^2*alfa_dot) + alfa_dot * x1
    // alfa in [0, 1]
    if (alfa < 0.0)
      return x0;
    else if (alfa > 1.0)
      return x1;
    else {
      if (std::abs(x0) < 1e-6) { // can not devide by zero
        return pos_vel_interpolation_only_x1(alfa, alfa_dot, x0, x1, v0, v1);
      }
      double c1 = - (v0 / alfa_dot - x1) / x0;
      double c2 = 3 -2*c1 + (v1 / alfa_dot - x1) / x0;
      double c3 = 1 - c1 - c2;
      return (1 - c1*alfa - c2*alfa*alfa - c3*alfa*alfa*alfa)*x0 + alfa*x1;
    }
  }

  // moving foot back to equilbrium
  inline double foot_pos_xy_generator(double q, double phase_shift,
                               double radial_frequency,
                               double delta_xFinal,
                               double forward_displacement,
                               double v0,
                               double v1,
                               int legsNum) {
    q = q + phase_shift;
    q = q - std::floor(q / (2 * M_PI)) * 2 * M_PI;
    double epsilon = M_PI / legsNum;

    if (q < 2 * (M_PI - epsilon)) {
      return delta_xFinal;
    } else if (q < 2 * M_PI) {
      double u = q - 2 * epsilon * (legsNum - 1);
      double alfa = u / (2 * epsilon);
      double alfa_dot = radial_frequency / (2 * epsilon);
      /* This reverts to simple_pos_interpolation
      double v = (-delta_xFinal + forward_displacement) * alfa_dot;
      double result = pos_vel_interpolation(alfa, alfa_dot, delta_xFinal, forward_displacement, v, v);
      */
      double result = pos_vel_interpolation(alfa, alfa_dot, delta_xFinal, forward_displacement, v0, v1);
      return result;
    } else {
      return 0.0;
    }
  }

  inline auto init_phase_shift(int feet_num) -> std::vector<double> {
    std::vector<double> phase_shift_vec(feet_num);
    for (int i = 0; i < feet_num; ++i) {
      phase_shift_vec[i] = i * (2 * M_PI / feet_num);
    }
    return phase_shift_vec;
  }

  inline auto invertQuaternion(const geometry_msgs::msg::Quaternion &q) -> geometry_msgs::msg::Quaternion{
    geometry_msgs::msg::Quaternion q_inv;
    q_inv.x = -q.x;
    q_inv.y = -q.y;
    q_inv.z = -q.z;
    q_inv.w = q.w;
    return q_inv;
  }

} // namespace penta_pod::kin::gait_generator

#endif // PENTA_POD_KIN_COMMONS_GAIT_UTILS_HPP_
