/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/solver.h"

namespace registration {

QGM::QGM(const size_t& max_iteration, const double& tolerance, const double& threshold_c) {
  this->max_iter = max_iteration;
  this->tol = tolerance;
  this->c = threshold_c;
}

Eigen::MatrixXd QGM::updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x) {
  mat_w = Eigen::MatrixXd::Zero(13, 13);
  for (auto& mat_i : terms) {
    mat_w +=
        mat_i / ((x.transpose() * mat_i * x + this->c * this->c) * (x.transpose() * mat_i * x + this->c * this->c));
  }
  return mat_w;
}

Eigen::VectorXd QGM::updateVariable(Eigen::MatrixXd& mat_w) {
  temp = mat_w.ldlt().solve(e);
  return (1 / (e.transpose() * temp)) * temp;
}

Eigen::Matrix4d QGM::solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) {
  std::vector<Eigen::MatrixXd> terms = compute_residual_terms(pcd1, pcd2, noise_bound * noise_bound);

  Eigen::Matrix4d init_mat = compute_initial_guess(pcd1, pcd2);
  x = se3_mat_to_vec(init_mat);

  mat_w = Eigen::MatrixXd::Zero(13, 13);
  for (auto& mat_i : terms) {
    mat_w += mat_i;
  }

  double prev_cost = x.transpose() * mat_w * x;
  double cost = 0.0;

  for (int i = 0; i < this->max_iter; i++) {
    // weight update
    mat_w = updateWeight(terms, x);
    // variable update
    x = updateVariable(mat_w);

    // stopping criteria
    cost = x.transpose() * mat_w * x;
    if (std::fabs(cost - prev_cost) / std::max(prev_cost, 1e-7) < this->tol) {
      break;
    }
    prev_cost = cost;
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

FracGM::FracGM(const size_t& max_iteration, const double& tolerance, const double& threshold_c) {
  this->max_iter = max_iteration;
  this->tol = tolerance;
  this->c = threshold_c;
}

std::vector<double> FracGM::updateAuxiliaryVariables(std::vector<Fractional>* terms) {
  std::vector<double> vec_1;  // beta
  std::vector<double> vec_2;  // mu

  for (auto& term : *terms) {
    vec_1.push_back(term.f() / term.h());
    vec_2.push_back(1 / term.h());
  }

  vec_1.insert(vec_1.end(), vec_2.begin(), vec_2.end());
  return vec_1;
}

Eigen::MatrixXd FracGM::updateWeight(std::vector<double>* alpha, std::vector<Fractional>* terms) {
  mat_w = Eigen::MatrixXd::Zero(13, 13);
  int n = terms->size();

  for (int i = 0; i < n; i++) {
    mat_w += alpha->at(n + i) * terms->at(i).f_mat() - alpha->at(n + i) * alpha->at(i) * terms->at(i).h_mat();
  }
  return mat_w;
}

Eigen::VectorXd FracGM::updateVariable(Eigen::MatrixXd& mat_w) {
  temp = mat_w.ldlt().solve(e);
  return (1 / (e.transpose() * temp)) * temp;
}

float FracGM::compute_psi_norm(std::vector<double>* alpha, std::vector<Fractional>* terms) {
  float a;
  float b;
  int n = terms->size();

  float loss = 0.0;
  for (int i = 0; i < n; i++) {
    a = -terms->at(i).f() + alpha->at(i) * terms->at(i).h();
    b = -1.0 + alpha->at(n + i) * terms->at(i).h();
    loss += a * a + b * b;
  }
  return sqrt(loss);
}

void FracGM::update_terms_cache(std::vector<Fractional>* terms, Eigen::VectorXd* vec) {
  for (auto& term : *terms) {
    term.update_cache(*vec);
  }
}

Eigen::Matrix4d FracGM::solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) {
  std::vector<Fractional> terms = compute_fractional_terms(pcd1, pcd2, noise_bound * noise_bound, this->c * this->c);

  Eigen::Matrix4d init_mat = compute_initial_guess(pcd1, pcd2);
  x = se3_mat_to_vec(init_mat);

  for (int i = 0; i < this->max_iter; i++) {
    // alternating solve alpha
    alpha = updateAuxiliaryVariables(&terms);
    mat_w = updateWeight(&alpha, &terms);

    // alternating solve x
    x = updateVariable(mat_w);
    update_terms_cache(&terms, &x);

    // stopping criteria
    psi_norm = compute_psi_norm(&alpha, &terms);
    if (psi_norm < this->tol) {
      break;
    }
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

}  // namespace registration
