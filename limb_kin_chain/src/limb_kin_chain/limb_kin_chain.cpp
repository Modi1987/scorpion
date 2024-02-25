#include <rclcpp/executors.hpp>

// include librarries
#include "limb_kin_chain/limb_kin_chain.hpp"  
#include "rclcpp/rclcpp.hpp"     

#include <vector>
#define pi 3.141592

namespace penta_pod::kin::limb_kin_chain {

  void Limb::init(const int n, const std::vector<double>& a, 
            const std::vector<double>& d, const std::vector<double>& alfa) {
    // store kinenatic constatnts
    this->dof = n;
    this->a = get_vector(n, a);
    this->d = get_vector(n, d);
    this->alfa = get_vector(n, alfa);
    // kinematics variables
    q = std::vector<double>(n);
    // ToDo initialize q to be equal to q0
    for(int i = 0; i < n; i++)q[i] = 0.; 
    J = std::vector<std::vector<double>>(3);
    for (int i = 0; i < 3; i++) {
      J[i] = std::vector<double>(n);
    }
    T = std::vector<std::vector<std::vector<double>>>(4);
    for (int i = 0; i < 4; i++) {
      T[i] = std::vector<std::vector<double>>(4);
      for (int j = 0; j < 4; j++) {
        T[i][j] = std::vector<double>(n);
      }
    }
    Ttemp = std::vector<std::vector<double>>(4);
    for (int i = 0; i < 4; i++) {
      Ttemp[i] = std::vector<double>(4);
    }
    xyz = std::vector<double>(3);
  }

  std::vector<double> Limb::get_vector(const int n, const std::vector<double>& vec) {
      std::vector<double> x(n);
      for(int i=0; i<n; i++){
          x[i] = vec[i];
    }
    return x;
  }

  void Limb::fk() {
    calculate_Ttemp_at_i(0);
    for(int i = 0; i < 4; i++) {
        for(int j = 0; j < 4; j++) {
          T[i][j][0] = Ttemp[i][j];
      }
    }
    for(int m=1; m<dof; m++){
      calculate_Ttemp_at_i(m);
      for(int i = 0; i<4; i++){
        for(int j = 0; j<4; j++){
          int accum = 0.;
          for(int k=0; k<4; k++){
            accum = accum + this->T[i][k][ m-1]*this->Ttemp[k][j];
          }
          this->T[i][j][m] = accum;
        }
      }		
    }
    for(int i=0; i<3; i++)xyz[i]=T[i][3][dof-1];
    for(int j=0; j<dof; j++) {
      double lx = xyz[0] - T[0][3][j];
      double ly = xyz[1] - T[1][3][j];
      double lz = xyz[2] - T[2][3][j];
      double kx = T[0][2][j];
      double ky = T[1][2][j];
      double kz = T[2][2][j];
      J[0][j] = -ly*kz + lz*ky;
      J[1][j] = +lx*kz - lz*kx;
      J[2][j] = -lx*ky + ly*kx;
    }
  }

  void Limb::calculate_Ttemp_at_i(int i) {
    double calpha = cos(alfa[i]);
    double sinalpha = sin(alfa[i]);
    double coshteta = cos(q[i]);
    double sintheta = sin(q[i]);
    // first column
    this->Ttemp[0][0] = coshteta;
    this->Ttemp[1][0] = sintheta*calpha;
    this->Ttemp[2][0] = sintheta*sinalpha;
    this->Ttemp[3][0] = 0.;
    // second column
    this->Ttemp[0][1] = -sintheta;
    this->Ttemp[1][1] = coshteta*calpha;
    this->Ttemp[2][1] = coshteta*sinalpha;
    this->Ttemp[3][1] = 0.;
	  // third column				
    this->Ttemp[0][2] = 0.;
    this->Ttemp[1][2] = -sinalpha;
    this->Ttemp[2][2] = calpha;
    this->Ttemp[3][2] = 0.;
	  // forth column
    this->Ttemp[0][3] = a[i];
    this->Ttemp[1][3] = -sinalpha*d[i];
    this->Ttemp[2][3] = calpha*d[i];
	  this->Ttemp[2][3] = 1.;
  }


  std::vector<double> Limb::get_ik(const double& x, const double& y, const double& z) {
    /* following is for not to get errors */
    for(int iterations = 0; iterations < 10; iterations++) {
      this->fk(); // calculates T and xyz and J @(q)
      double c = 0.5;
      double dx = c*(x - xyz[0]);
      double dy = c*(y - xyz[1]);
      double dz = c*(z - xyz[2]);
      for(int j = 0; j < dof; j++) {
        q[j] = q[j] + dx*J[0][j] + dy*J[1][j] + dz*J[2][j];
      }
    }
    std::vector<double> jpos;
    for(int i = 0; i < dof; i++) {
      jpos.push_back(q[i]);
    }
    return jpos;
  }
}  // penta_pod::kin::limb_kin_chain
