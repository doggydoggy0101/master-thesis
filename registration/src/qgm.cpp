/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/qgm.h"

namespace registration {

qgm::qgm(const size_t& max_iteration, const double& tol, const double& c, const double& noise_bound) {
  max_iteration_ = max_iteration;
  tol_ = tol;
  c_ = c;
  noise_bound_ = noise_bound;
}

Eigen::MatrixXd qgm::updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x) {
  mat_w = Eigen::MatrixXd::Zero(13, 13);
  for (auto& mat_i : terms) {
    mat_w += mat_i / ((x.transpose() * mat_i * x + c_ * c_) * (x.transpose() * mat_i * x + c_ * c_));
  }
  return mat_w;
}

Eigen::VectorXd qgm::updateVariable(Eigen::MatrixXd& mat_w) {
  temp = mat_w.ldlt().solve(e);
  return (1 / (e.transpose() * temp)) * temp;
}

Eigen::Matrix4d qgm::solve(const PointCloud& pcd1, const PointCloud& pcd2) {
  std::vector<Eigen::MatrixXd> terms = compute_residual_terms(pcd1, pcd2, noise_bound_ * noise_bound_);

  Eigen::Matrix4d init_mat = compute_initial_guess(pcd1, pcd2);
  x = se3_mat_to_vec(init_mat);

  mat_w = Eigen::MatrixXd::Zero(13, 13);
  for (auto& mat_i : terms) {
    mat_w += mat_i;
  }

  double prev_cost = x.transpose() * mat_w * x;
  double cost = 0.0;

  for (int i = 0; i < max_iteration_; i++) {
    // weight update
    mat_w = updateWeight(terms, x);
    // variable update
    x = updateVariable(mat_w);

    // stopping criteria
    cost = x.transpose() * mat_w * x;
    if (std::fabs(cost - prev_cost) / std::max(prev_cost, 1e-7) < tol_) {
      break;
    }
    prev_cost = cost;
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

}  // namespace registration
