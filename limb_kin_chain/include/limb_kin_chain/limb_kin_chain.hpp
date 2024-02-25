#ifndef LIMB_KIN_CHAIN_HPP_
#define LIMB_KIN_CHAIN_HPP_

#include <rclcpp/executors.hpp>
#include "rclcpp/rclcpp.hpp"                      // for rclcpp
#include <vector>

namespace penta_pod::kin::limb_kin_chain {
  /*
  * Kinematic chain solver using FK and IK using DLS
  * to use using following calls:
  * - init: to initialize with modified DH
  * - get_ik: to calculate inverse kinematics
  */
  class Limb {
    private:
      // kinematic variables
      std::vector<double> xyz;
      std::vector<double> q;
      std::vector<std::vector<double>> J; 
      std::vector<std::vector<std::vector<double>>> T;
      std::vector<std::vector<double>> Ttemp;
      // kinematic constants
      int dof;
      std::vector<double> a;
      std::vector<double> d;
      std::vector<double> alfa;

    public:
      explicit Limb(){};
      void init(const int n, const std::vector<double>& a, const std::vector<double>& d, const std::vector<double>& alfa);
      void fk();
      void ik();
      std::vector<double> get_ik(const double& x, const double& y, const double& z);
      std::vector<double> get_vector(const int n, const std::vector<double>& vec);
      void calculate_Ttemp_at_i(int i);
  }; 
  
} // namespace penta_pod::kin::limb_kin_chain

#endif  // LIMB_KIN_CHAIN_HPP_