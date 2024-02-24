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
      double *xyz;
      int dof;
      double *q;
      double **J;
      double ***T;
      double **Ttemp;
      // kinematic constants
      double *a;
      double *d;
      double *alfa;

    public:
      explicit Limb(){};
      ~Limb();
      void init(int n, std::vector<double> a, std::vector<double> d, std::vector<double> alfa);
      void fk();
      void ik();
      std::vector<double> get_ik(double x, double y, double z);
      double* get_vector(int n, std::vector<double> vec);
      void calculate_Ttemp_at_i(int i);
  }; 
  
} // namespace penta_pod::kin::limb_kin_chain

#endif  // LIMB_KIN_CHAIN_HPP_