#include <rclcpp/executors.hpp>

// include librarries
#include "limb_kin_chain/limb_kin_chain.hpp"
#include "rclcpp/rclcpp.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#define DEBUG_PRINTS false

static long malloc_start_log_window_millis = 5000;
static long malloc_end_log_window_millis = 5500;
long start_time_millis = -1;
bool malloc_logging_enabled = false;
long current_time_millis() {
  return std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::system_clock::now().time_since_epoch())
      .count();
}

void *operator new(size_t size) {
  if (start_time_millis < 0) {
    start_time_millis = current_time_millis();
  }
  auto delta_millis = current_time_millis() - start_time_millis;
  if (delta_millis >= malloc_start_log_window_millis &&
      delta_millis <= malloc_end_log_window_millis) {
    malloc_logging_enabled = true;
  } else {
    malloc_logging_enabled = false;
  }
  if (malloc_logging_enabled) {
    std::cout << "mallocating: " << size << " bytes\n";
  }
  void *p = malloc(size);
  if (!p)
    throw std::bad_alloc();
  return p;
}

namespace penta_pod::kin::limb_kin_chain {

inline void get_transform(double q_i, double alfa_i, double a_i, double d_i,
                          int index, std::vector<std::vector<double>> &Ttemp) {
  double c_alfa = cos(alfa_i);
  double s_alfa = sin(alfa_i);
  double c_theta = cos(q_i);
  double s_theta = sin(q_i);
  // first column
  Ttemp[0][0] = c_theta;
  Ttemp[1][0] = s_theta * c_alfa;
  Ttemp[2][0] = s_theta * s_alfa;
  Ttemp[3][0] = 0.;
  // second column
  Ttemp[0][1] = -s_theta;
  Ttemp[1][1] = c_theta * c_alfa;
  Ttemp[2][1] = c_theta * s_alfa;
  Ttemp[3][1] = 0.;
  // third column
  Ttemp[0][2] = 0.;
  Ttemp[1][2] = -s_alfa;
  Ttemp[2][2] = c_alfa;
  Ttemp[3][2] = 0.;
  // forth column
  Ttemp[0][3] = a_i;
  Ttemp[1][3] = -s_alfa * d_i;
  Ttemp[2][3] = c_alfa * d_i;
  Ttemp[3][3] = 1.;
  if (DEBUG_PRINTS) {
    std::cout << "T_" << index << "_" << index - 1 << std::endl;
    for (int k = 0; k < 4; k++) {
      for (int l = 0; l < 4; l++) {
        std::cout << Ttemp[k][l] << " | ";
      }
      std::cout << std::endl;
    }
  }
}

void Limb::init(const int n, const std::vector<double> &a,
                const std::vector<double> &d, const std::vector<double> &alfa,
                const std::vector<double> &eef_trans,
                const std::vector<double> &q_max,
                const std::vector<double> &q_min) {
  size_t n_size = static_cast<size_t>(n);
  if (a.size() < n_size || d.size() < n_size || alfa.size() < n_size ||
      eef_trans.size() < 3 || q_max.size() < n_size || q_min.size() < n_size) {
    throw std::invalid_argument("Input vector size is incorrect");
  }
  // store kinenatic constatnts
  this->dof = n;
  this->a = get_vector(n, a);
  this->d = get_vector(n, d);
  this->alfa = get_vector(n, alfa);
  this->eef_trans = get_vector(3, eef_trans);
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
  // kinematics variables
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
  tcp_xyz_base = std::vector<double>(3);
  JJT = std::vector<std::vector<double>>(3);
  for (int i = 0; i < 3; i++) {
    JJT[i] = std::vector<double>(3);
  }
}

std::array<std::array<double, 3>, 3> Limb::JJT_dls_inverter(double lambda) {
  std::array<std::array<double, 3>, 3> m = {
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

  std::array<std::array<double, 3>, 3> minv = {
      {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}};

  // Add damping
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      if (i == j) {
        m[i][j] = JJT[i][j] + lambda;
      } else {
        m[i][j] = JJT[i][j];
      }
    }
  }
  // computes the inverse of a matrix m
  double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
               m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
               m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

  double invdet = 1 / det;

  minv[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * invdet;
  minv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * invdet;
  minv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * invdet;
  minv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * invdet;
  minv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * invdet;
  minv[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * invdet;
  minv[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * invdet;
  minv[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * invdet;
  minv[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * invdet;
  return minv;
}

std::vector<double> Limb::get_vector(const int n,
                                     const std::vector<double> &vec) {
  std::vector<double> x(n);
  for (int i = 0; i < n; i++) {
    x[i] = vec[i];
  }
  return x;
}

void Limb::fk(const std::vector<double> &q) {
  if (malloc_logging_enabled) {
    std::cout << "=== ENTERING fk ===" << std::endl;
  }
  constexpr int start_index = 0;
  get_transform(q[start_index], this->alfa[start_index], this->a[start_index],
                this->d[start_index], start_index,
                this->Ttemp); // result is in Ttemp
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      T[i][j][0] = Ttemp[i][j];
    }
  }
  for (int m = 1; m < this->dof; m++) { // loop over all T matrices
    get_transform(q[m], this->alfa[m], this->a[m], this->d[m], m,
                  this->Ttemp); // result is in Ttemp

    for (int i = 0; i < 4; i++) {   // loop over rows of matrix T[m]
      for (int j = 0; j < 4; j++) { // loop over columns of matrix T[m]
        double accum = 0.;
        for (int k = 0; k < 4; k++) {
          accum = accum + this->T[i][k][m - 1] * this->Ttemp[k][j];
        }
        this->T[i][j][m] = accum;
      }
    }
  }
  if (DEBUG_PRINTS) {
    for (int m = 0; m < this->dof; m++) {
      std::cout << "T_" << m << "_-1" << std::endl;
      for (int k = 0; k < 4; k++) {
        for (int l = 0; l < 4; l++) {
          std::cout << T[k][l][m] << " | ";
        }
        std::cout << std::endl;
      }
    }
  }
  if (DEBUG_PRINTS) {
    for (int i = 0; i < 3; i++)
      std::cout << "flange_pos[" << i << "]: " << this->T[i][3][this->dof - 1]
                << " ";
    std::cout << std::endl;
  }
  for (int i = 0; i < 3; i++) {
    double accum = 0.;
    for (int j = 0; j < 3; j++) {
      accum = accum + this->T[i][j][this->dof - 1] * this->eef_trans[j];
    }
    this->tcp_xyz_base[i] = accum + this->T[i][3][this->dof - 1];
  }
  if (DEBUG_PRINTS) {
    for (int i = 0; i < 3; i++)
      std::cout << "pos[" << i << "]: " << this->tcp_xyz_base[i] << " ";
    std::cout << std::endl;
  }
  // calculate J
  for (int j = 0; j < this->dof; j++) {
    double lx = tcp_xyz_base[0] - T[0][3][j];
    double ly = tcp_xyz_base[1] - T[1][3][j];
    double lz = tcp_xyz_base[2] - T[2][3][j];
    double kx = T[0][2][j];
    double ky = T[1][2][j];
    double kz = T[2][2][j];
    this->J[0][j] = -ly * kz + lz * ky;
    this->J[1][j] = +lx * kz - lz * kx;
    this->J[2][j] = -lx * ky + ly * kx;
  }

  if (DEBUG_PRINTS) {
    std::cout << "J :" << std::endl;
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < this->dof; j++)
        std::cout << J[i][j] << " ";
      std::cout << std::endl;
    }
  }
  // calculate JJT
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      double accum = 0.;
      for (int k = 0; k < this->dof; k++) {
        accum = accum + J[i][k] * J[j][k];
      }
      JJT[i][j] = accum;
      JJT[j][i] = accum;
    }
  }
}

bool Limb::get_ik(const double &x, const double &y, const double &z,
                  const std::vector<double> &q0, std::vector<double> &q_out) {
  if (malloc_logging_enabled) {
    std::cout << "=== ENTERING get_ik ===" << std::endl;
  }
  if (q_out.size() < static_cast<size_t>(this->dof)) {
    std::cerr << "Size of vector (q_out) is incorrect" << std::endl;
    return false;
  }
  /* following is for not to get errors */
  constexpr int max_iterations = 200;
  constexpr double epsilon = 0.0001; // positioning accuracy of 0.1 mm
  for (int i = 0; i < this->dof; i++)
    q_out[i] = q0[i];
  for (int iterations = 0; iterations < max_iterations; iterations++) {
    this->fk(q_out); // calculates (T, tcp_xyz_base, J, JJT) @(q)
    double lambda = 0.1;
    auto JJT_1 = JJT_dls_inverter(lambda);
    double c = 0.9; // (iterations+1)/max_iterations;
    double dx = (x - tcp_xyz_base[0]);
    double dy = (y - tcp_xyz_base[1]);
    double dz = (z - tcp_xyz_base[2]);
    double norm_seq = dx * dx + dy * dy + dz * dz;
    if (norm_seq < epsilon * epsilon)
      break;
    std::array<double, 3> disp = {c * dx, c * dy, c * dz};
    std::array<double, 3> disp1{};
    for (int i = 0; i < 3; i++) {
      double accum = 0.;
      for (int j = 0; j < 3; j++)
        accum = accum + JJT_1[i][j] * disp[j];
      disp1[i] = accum;
    }
    for (int j = 0; j < this->dof; j++) {
      q_out[j] = q_out[j] + J[0][j] * disp1[0] + J[1][j] * disp1[1] +
                 J[2][j] * disp1[2];
      if (q_out[j] > q_max[j])
        q_out[j] = q_max[j];
      if (q_out[j] < q_min[j])
        q_out[j] = q_min[j];
    }
  }
  return true;
}
} // namespace penta_pod::kin::limb_kin_chain
