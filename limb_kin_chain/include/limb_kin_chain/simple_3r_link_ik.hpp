#ifndef SIMPLE_3R_LINK_IK_HPP_
#define SIMPLE_3R_LINK_IK_HPP_

#include "rclcpp/rclcpp.hpp" // for rclcpp
#include <rclcpp/executors.hpp>
#include <vector>
#include "limb_kin_chain/limb_ik_interface.hpp"

namespace penta_pod::kin::limb_kin_chain {
/*
 * Kinematic chain solver using FK and IK using DLS
 * to use using following calls:
 * - init: to initialize with modified DH
 * - get_ik: to calculate inverse kinematics
 */

template <class T>
constexpr int sign(T x) noexcept {
    return (x >= 0) ? 1 : -1;
};

class Simple3RLinkLimb : public LimbIKInterface {
private:

  // kinematic constants
  int dof;
  std::vector<double> a;
  std::vector<double> d;
  std::vector<double> alfa;
  std::vector<double> eef_trans;

  std::vector<double> q_max;
  std::vector<double> q_min;

  std::vector<double> get_vector(const int n,
                                     const std::vector<double> &vec);

public:
  explicit Simple3RLinkLimb(){};
  void init(const int n, const std::vector<double> &a,
            const std::vector<double> &d, const std::vector<double> &alfa,
            const std::vector<double> &eef_trans,
            const std::vector<double> &q_max, const std::vector<double> &q_min) override;
  std::vector<double> get_ik(const double &x, const double &y, const double &z,
                             const std::vector<double> &/*q0*/) override;
};

} // namespace penta_pod::kin::limb_kin_chain

#endif // SIMPLE_3R_LINK_IK_HPP_