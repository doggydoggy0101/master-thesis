/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/gnc.h"

namespace registration {

namespace gnc {

std::vector<Eigen::MatrixXd> compute_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2) {
  std::vector<Eigen::MatrixXd> terms;
  terms.reserve(pcd1.rows());

  Eigen::Matrix3d id3 = Eigen::Matrix3d::Identity(3, 3);
  Eigen::MatrixXd mat_n;
  Eigen::MatrixXd mat_m;

  for (int i = 0; i < pcd1.rows(); i++) {
    mat_n = Eigen::Matrix<double, 3, 13>::Zero(3, 13);
    mat_n.block<3, 9>(0, 0) = kroneckerProduct(pcd1.row(i), id3);
    mat_n.block<3, 3>(0, 9) = id3;
    mat_n.block<3, 1>(0, 12) = -pcd2.row(i);

    mat_m = (mat_n.transpose() * mat_n) / (noise_bound_2);
    terms.push_back(mat_m);
  }
  return terms;
}

double compute_initial_mu(double& res_sq, std::string& robust, double& c) {
  double mu = 0.0;
  if (robust == "TLS") {
    mu = (c * c) / (2 * res_sq - c * c);
    // if the residual is very large, set threshold of 1e-6 to avoid mu = 0
    if (mu >= 0 && mu < 1e-6) {
      mu = 1e-6;
    }
  } else if (robust == "GM") {
    mu = (2 * res_sq) / (c * c);
  } else if (robust == "L0") {
    mu = 1;
  }
  return mu;
}

void update_mu(double& mu, std::string& robust, double& gnc_factor, double& threshold_c, bool& superlinear) {
  if (robust == "TLS") {
    if (superlinear && mu < 1) {  // Peng's superlinear surrogate update
      mu = std::sqrt(mu);
    }
    mu *= gnc_factor;
  } else if (robust == "GM") {
    mu /= gnc_factor;
    mu = std::max(1.0, mu);  // Saturate at 1
  } else if (robust == "L0") {
    std::max(gnc_factor * mu * mu, threshold_c);  // we write epsilon as mu and beta as gnc_factor
  }
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x,
                                                         std::string& robust, double& c, double& mu,
                                                         bool& majorization) {
  Eigen::MatrixXd mat_w = Eigen::MatrixXd::Zero(13, 13);
  Eigen::VectorXd vec_w = Eigen::VectorXd::Zero(terms.size());

  if (robust == "TLS") {
    if (majorization) {  // Peng's majorization surrogate
      for (size_t i = 0; i < terms.size(); i++) {
        double res = x.transpose() * terms[i] * x;
        if (res <= c * c) {
          mat_w += terms[i];
          vec_w[i] = 1.0;
        } else if (res < ((mu + 1) / mu) * ((mu + 1) / mu) * c * c) {
          // Both my derivation and their implementation indicate there should be a square root in Eq.(17).
          double w_i = c * (mu + 1) / std::sqrt(res) - mu;
          mat_w += w_i * terms[i];
          vec_w[i] = w_i;
        }
      }
    } else {  // Yang's surrogate in GNC
      for (size_t i = 0; i < terms.size(); i++) {
        double res = x.transpose() * terms[i] * x;
        if (res <= (mu / (mu + 1)) * c * c) {
          mat_w += terms[i];
          vec_w[i] = 1.0;
        } else if (res <= ((mu + 1) / mu) * c * c) {
          double w_i = c * std::sqrt(mu * (mu + 1) / res) - mu;
          mat_w += w_i * terms[i];
          vec_w[i] = w_i;
        }
      }
    }
  } else if (robust == "GM") {
    for (size_t i = 0; i < terms.size(); i++) {
      double res = x.transpose() * terms[i] * x;
      double w_i = std::pow((mu * c * c) / (res + mu * c * c), 2);
      mat_w += w_i * terms[i];
      vec_w[i] = w_i;
    }
  } else if (robust == "L0") {
    for (size_t i = 0; i < terms.size(); i++) {
      double res = x.transpose() * terms[i] * x;
      double w_i = 1 / std::max(res, mu * mu);  // we write epsilon as mu
      mat_w += w_i * terms[i];
      vec_w[i] = w_i;
    }
  }
  return std::make_pair(mat_w, vec_w);
}

bool check_mu_convergence(double& mu, std::string& robust) {
  if (robust == "GM" && std::abs(mu - 1.0) < 1e-9) {
    return true;
  }
  return false;
}

bool check_weight_convergence(Eigen::VectorXd& weight, std::string& robust, double& weight_tol) {
  if (robust == "TLS") {
    for (int i = 0; i < weight.size(); i++) {
      if (std::abs(weight[i] - std::round(weight[i])) > weight_tol) {
        return false;
      }
    }
    return true;
  }
  return false;
}

};  // namespace gnc

};  // namespace registration