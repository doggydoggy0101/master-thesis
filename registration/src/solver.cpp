/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/solver.h"

namespace registration {

// Iterative Re-weighted Least Squares
IrlsSolver::IrlsSolver(const size_t& max_iteration, const double& tolerance, const std::string& robust_type, const double& threshold_c) {
  this->max_iter = max_iteration;
  this->tol = tolerance;
  this->robust = robust_type;
  this->c = threshold_c;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> IrlsSolver::updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x) {
  mat_w = Eigen::MatrixXd::Zero(13, 13);
  vec_w = Eigen::VectorXd::Zero(terms.size());

  if (this->robust == "TLS") {
    for (size_t i = 0; i < terms.size(); i++) {
      if (x.transpose() * terms[i] * x <= this->c * this->c) {
        mat_w += terms[i];
        vec_w[i] = 1.0;
      }
    }
  } else if (this->robust == "GM") {
    for (size_t i = 0; i < terms.size(); i++) {
      double w_i = 1.0 / ((x.transpose() * terms[i] * x + this->c * this->c) * (x.transpose() * terms[i] * x + this->c * this->c));
      mat_w += w_i * terms[i];
      vec_w[i] = w_i;
    }
  }
  return std::make_pair(mat_w, vec_w);
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
    auto [mat_w, vec_w] = updateWeight(terms, x);
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

// Fractional program for Geman-McClure
FracgmSolver::FracgmSolver(const size_t& max_iteration, const double& tolerance, const double& threshold_c) {
  this->max_iter = max_iteration;
  this->tol = tolerance;
  this->c = threshold_c;
}

std::vector<double> FracgmSolver::updateAuxiliaryVariables(std::vector<fracgm::Fractional>* terms) {
  std::vector<double> vec_1;  // beta
  std::vector<double> vec_2;  // mu

  for (auto& term : *terms) {
    vec_1.push_back(term.f() / term.h());
    vec_2.push_back(1 / term.h());
  }

  vec_1.insert(vec_1.end(), vec_2.begin(), vec_2.end());
  return vec_1;
}

Eigen::MatrixXd FracgmSolver::updateWeight(std::vector<double>* alpha, std::vector<fracgm::Fractional>* terms) {
  mat_w = Eigen::MatrixXd::Zero(13, 13);
  int n = terms->size();

  for (int i = 0; i < n; i++) {
    mat_w += alpha->at(n + i) * terms->at(i).f_mat() - alpha->at(n + i) * alpha->at(i) * terms->at(i).h_mat();
  }
  return mat_w;
}

Eigen::Matrix4d FracgmSolver::solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) {
  std::vector<fracgm::Fractional> terms = fracgm::compute_terms(pcd1, pcd2, noise_bound * noise_bound, this->c * this->c);

  Eigen::Matrix4d init_mat = Eigen::umeyama(pcd1.transpose(), pcd2.transpose(), false);
  x = se3_mat_to_vec(init_mat);

  for (int i = 0; i < this->max_iter; i++) {
    // alternating solve alpha
    alpha = FracgmSolver::updateAuxiliaryVariables(&terms);
    mat_w = FracgmSolver::updateWeight(&alpha, &terms);

    // alternating solve x
    x = solveQuadraticProgram(mat_w);
    fracgm::update_terms_cache(&terms, &x);

    // stopping criteria
    psi_norm = fracgm::compute_psi_norm(&alpha, &terms);
    if (psi_norm < this->tol) {
      break;
    }
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

}  // namespace registration
