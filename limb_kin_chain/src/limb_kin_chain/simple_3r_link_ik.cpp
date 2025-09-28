
#include "rclcpp/rclcpp.hpp" // for rclcpp
#include <rclcpp/executors.hpp>
#include <iostream>
#include <vector>
#include <math.h>
#include "limb_kin_chain/simple_3r_link_ik.hpp"

namespace penta_pod::kin::limb_kin_chain {

void Simple3RLinkLimb::init(const int n, const std::vector<double> &a,
                const std::vector<double> &d, const std::vector<double> &alfa,
                const std::vector<double> &eef_trans,
                const std::vector<double> &q_max,
                const std::vector<double> &q_min) {
  size_t n_size = static_cast<size_t>(n);
  std::cout << "Trying to initialize analytic IK Simple3RLinkLimb!" << std::endl;
  if (a.size() < n_size || d.size() < n_size || alfa.size() < n_size ||
          eef_trans.size() < 3 || q_max.size() < n_size ||
      q_min.size() < n_size) {
    throw std::invalid_argument("Input vector size is incorrect");
  }
  // store kinenatic constatnts
  this->dof = n;
  if (this->dof != 3) {
    throw std::invalid_argument("This IK solver only works for 3R manipulators.");
  }
  // store kinematic constants
  this->a = get_vector(n, a);
  if (std::abs(a[0])>1e-5) {
    throw std::invalid_argument("This IK solver only works for a[0] = 0.");
  }
  this->d = get_vector(n, d);
  // check if d is zero for all links
  for (const auto &val : d) {
    if (std::abs(val) > 1e-5) {
      throw std::invalid_argument("This IK solver only works for planar 3R "
                                  "manipulators with d=0 for all links.");
    }
  }
  this->alfa = get_vector(n, alfa);
  // check if alfa is valid
  if (std::abs(alfa[0]) > 1e-5) {
    throw std::invalid_argument("This IK solver only works for alfa[0] = 0.");
  }
  double alfa_1_value = -1.570796;
  if (std::abs(alfa[1] - alfa_1_value) > 1e-5) {
      std::string message = "This IK solver only works for alfa[1] = " 
                            + std::to_string(alfa_1_value);
      throw std::invalid_argument(message);
  }
  if (std::abs(alfa[2]) > 1e-5) {
    throw std::invalid_argument("This IK solver only works for alfa[2] = 0.");
  }

  this->eef_trans = get_vector(3, eef_trans);
  if (std::abs(eef_trans[1]) > 1e-5) {
    throw std::invalid_argument("This IK solver only works for eef_trans[1] = 0.");
  }
  if (std::abs(eef_trans[2]) > 1e-5) {
    throw std::invalid_argument("This IK solver only works for eef_trans[2] = 0.");
  }

  this->q_max = get_vector(n, q_max);
  this->q_min = get_vector(n, q_min);
  // print some useful info
  std::cout << "Limb initialized with parameters" << std::endl;
  for (int i = 0; i < this->dof; i++) {
    std::cout << "m_dh(" << i << "): "
              << " | a: " << this->a[i] << " | d:" << this->d[i]
              << " | alfa: " << this->alfa[i] << std::endl;
  }
  for (int i = 0; i < 3; i++) {
    std::cout << "eef_trans[" << i << "]: " << this->eef_trans[i] << std::endl;
  }
}

std::vector<double> Simple3RLinkLimb::get_ik(const double &x, const double &y, const double &z,
            const std::vector<double> &/*q0*/) {
    double l1 = a[1];
    double l2 = a[2];
    double l3 = eef_trans[0];

    double temp = std::sqrt(x*x + y*y) - l1;
    double w = std::sqrt(temp*temp + z*z);

    double q1 = atan2(y, x);
    double q3 = acos((w*w - l2*l2 - l3*l3) / (2*l2*l3));
    double q2 = atan2(-z, temp) - atan2(l3*sin(q3), l2 + l3*cos(q3));

    return std::vector<double>{q1, q2, q3};
}

std::vector<double> Simple3RLinkLimb::get_vector(const int n,
                                     const std::vector<double> &vec) {
  std::vector<double> x(n);
  for (int i = 0; i < n; i++) {
    x[i] = vec[i];
  }
  return x;
}

} // namespace penta_pod::kin::limb_kin_chain
