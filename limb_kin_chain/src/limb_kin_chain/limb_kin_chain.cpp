#include <rclcpp/executors.hpp>

// include librarries
#include "limb_kin_chain/limb_kin_chain.hpp"  
#include "rclcpp/rclcpp.hpp"     

#include <vector>
#define pi 3.141592

namespace penta_pod::kin::limb_kin_chain {

  void Limb::init(int n, std::vector<double> a, std::vector<double> d, std::vector<double> alfa) {
    // store kinenatic constatnts
    this->dof = n;
    this->a = get_vector(n, a);
    this->d = get_vector(n, d);
    this->alfa = get_vector(n, alfa);
    // kinematics variables
    q = new double[dof];
    J = new double*[3];
    for (int i = 0; i < 3; ++i) {
      J[i] = new double[dof];
    }
    T = new double**[4];
    for (int i = 0; i < 4; ++i) {
      T[i] = new double*[4];
      for (int j = 0; j < 4; ++j) {
        T[i][j] = new double[dof];
      }
    }
    Ttemp = new double*[4];
    for (int i = 0; i < 4; ++i) {
      Ttemp[i] = new double[4];
    }
    xyz = new double[3];
  }

  double* Limb::get_vector(int n, std::vector<double> vec) {
      double *x = new double[n];
      for(int i=0; i<n; i++){
          x[i] = vec[i];
    }
    return x;
  }

  Limb::~Limb() {
    delete[] q;
    for (int i = 0; i < 3; ++i) {
      delete[] J[i];
    }
    delete[] J;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        delete[] T[i][j];
      }
      delete[] T[i];
    }
    delete[] T;
    for (int i = 0; i < 3; ++i) {
      delete[] Ttemp[i];
    }
    delete[] Ttemp;
    delete[] xyz;
    delete[] a;
    delete[] alfa;
    delete[] d;
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
  }

  void Limb::calculate_Ttemp_at_i(int i) {
    double a = this->a[i];
    double d = this->d[i];
    double alfa = this->alfa[i];
    double calpha = cos(alfa);
    double sinalpha = sin(alfa);
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
    this->Ttemp[0][3] = a;
    this->Ttemp[1][3] = -sinalpha*d;
    this->Ttemp[2][3] = calpha*d;
	  this->Ttemp[2][3] = 1.;
  }


  std::vector<double> Limb::get_ik(double x, double y, double z) {
    std::vector<double> jpos;
    for(int i = 0; i < dof; i++) {
      jpos.push_back(q[i]);
    }
    /* following is for not to get errors */
    jpos[0] = x;
    jpos[1] = y;
    jpos[3] = z;
    return jpos;
  }
}  // penta_pod::kin::limb_kin_chain
