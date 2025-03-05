/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/solver.h"

namespace registration {

// Iterative Re-weighted Least Squares
IrlsSolver::IrlsSolver(Params params) {
  this->max_iter = params.max_iteration;
  this->tol = params.tolerance;
  this->robust = params.robust_type;
  this->c = params.threshold_c;
}

Eigen::Matrix4d IrlsSolver::solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) {
  std::vector<Eigen::MatrixXd> terms = irls::compute_terms(pcd1, pcd2, noise_bound * noise_bound);

  Eigen::Matrix4d init_mat = Eigen::umeyama(pcd1.transpose(), pcd2.transpose(), false);
  x = se3_mat_to_vec(init_mat);

  mat_w = Eigen::MatrixXd::Zero(13, 13);
  for (auto& mat_i : terms) {
    mat_w += mat_i;
  }

  double prev_cost = x.transpose() * mat_w * x;
  double curr_cost = 0.0;

  for (int i = 0; i < this->max_iter; i++) {
    // weight update
    auto [mat_w, vec_w] = irls::updateWeight(terms, x, this->robust, this->c);
    // NOTE: TLS extreme outlier cases when all the data are outliers.
    if (vec_w.sum() == 0) {
      break;
    }
    // variable update
    x = solveQuadraticProgram(mat_w);
    // stopping criteria
    curr_cost = x.transpose() * mat_w * x;
    if (checkCostConvergence(prev_cost, curr_cost)) {
      break;
    }
    prev_cost = curr_cost;
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

// Graduated Non-Convexity
GncSolver::GncSolver(Params params) {
  this->max_iter = params.max_iteration;
  this->tol = params.tolerance;
  this->robust = params.robust_type;
  this->c = params.threshold_c;
  this->gnc_factor = params.gnc_factor;
  this->weight_tol = params.weight_tolerance;
  this->major = params.majorization;
  this->superlinear = params.superlinear;
}

Eigen::Matrix4d GncSolver::solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) {
  std::vector<Eigen::MatrixXd> terms = gnc::compute_terms(pcd1, pcd2, noise_bound * noise_bound);

  Eigen::Matrix4d init_mat = Eigen::umeyama(pcd1.transpose(), pcd2.transpose(), false);
  x = se3_mat_to_vec(init_mat);

  double max_res2 = 0.0;
  mat_w = Eigen::MatrixXd::Zero(13, 13);
  for (auto& mat_i : terms) {
    mat_w += mat_i;
    double res2 = x.transpose() * mat_i * x;
    if (res2 > max_res2) {
      max_res2 = res2;
    }
  }

  double prev_cost = x.transpose() * mat_w * x;
  double curr_cost = 0.0;
  mu = gnc::compute_initial_mu(max_res2, this->robust, this->c);

  for (int i = 0; i < this->max_iter; i++) {
    // weight update
    auto [mat_w, vec_w] = gnc::updateWeight(terms, x, this->robust, this->c, mu, this->major);
    // NOTE: TLS extreme outlier cases when all the data are outliers.
    if (vec_w.sum() == 0) {
      break;
    }
    // variable update
    x = solveQuadraticProgram(mat_w);
    // stopping criteria
    curr_cost = x.transpose() * mat_w * x;
    if (checkCostConvergence(prev_cost, curr_cost) || gnc::check_mu_convergence(mu, this->robust) ||
        gnc::check_weight_convergence(vec_w, this->robust, this->weight_tol)) {
      break;
    }
    prev_cost = curr_cost;

    gnc::update_mu(mu, this->robust, this->gnc_factor, this->c, this->superlinear);
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

// Fractional program for Geman-McClure
FracgmSolver::FracgmSolver(Params params) {
  this->max_iter = params.max_iteration;
  this->tol = params.tolerance;
  this->c = params.threshold_c;
}

Eigen::Matrix4d FracgmSolver::solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) {
  std::vector<fracgm::Fractional> terms =
      fracgm::compute_terms(pcd1, pcd2, noise_bound * noise_bound, this->c * this->c);

  Eigen::Matrix4d init_mat = Eigen::umeyama(pcd1.transpose(), pcd2.transpose(), false);
  x = se3_mat_to_vec(init_mat);

  for (int i = 0; i < this->max_iter; i++) {
    // alternating solve alpha
    alpha = fracgm::updateAuxiliaryVariables(terms);
    mat_w = fracgm::updateWeight(alpha, terms);

    // alternating solve x
    x = solveQuadraticProgram(mat_w);
    fracgm::update_terms_cache(terms, x);

    // stopping criteria
    psi_norm = fracgm::compute_psi_norm(alpha, terms);
    if (psi_norm < this->tol) {
      break;
    }
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

}  // namespace registration
