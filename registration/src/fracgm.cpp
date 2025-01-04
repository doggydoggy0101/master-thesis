/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/fracgm.h"

namespace registration {

fracgm::fracgm(const size_t &max_iteration, const double &tol, const double &c, const double &noise_bound) {
  max_iteration_ = max_iteration;
  tol_ = tol;
  c_ = c;
  noise_bound_ = noise_bound;
}

std::vector<double> fracgm::updateAuxiliaryVariables(std::vector<Fractional> *terms) {
  std::vector<double> vec_1;  // beta
  std::vector<double> vec_2;  // mu

  for (auto &term : *terms) {
    vec_1.push_back(term.f() / term.h());
    vec_2.push_back(1 / term.h());
  }

  vec_1.insert(vec_1.end(), vec_2.begin(), vec_2.end());
  return vec_1;
}

Eigen::MatrixXd fracgm::updateWeight(std::vector<double> *alpha, std::vector<Fractional> *terms) {
  mat_w = Eigen::MatrixXd::Zero(13, 13);
  int n = terms->size();

  for (int i = 0; i < n; i++) {
    mat_w += alpha->at(n + i) * terms->at(i).f_mat() - alpha->at(n + i) * alpha->at(i) * terms->at(i).h_mat();
  }
  return mat_w;
}

Eigen::VectorXd fracgm::updateVariable(Eigen::MatrixXd &mat_w) {
  temp = mat_w.ldlt().solve(e);
  return (1 / (e.transpose() * temp)) * temp;
}

float fracgm::compute_psi_norm(std::vector<double> *alpha, std::vector<Fractional> *terms) {
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

void fracgm::update_terms_cache(std::vector<Fractional> *terms, Eigen::VectorXd *vec) {
  for (auto &term : *terms) {
    term.update_cache(*vec);
  }
}

Eigen::Matrix4d fracgm::solve(const PointCloud &pcd1, const PointCloud &pcd2) {
  std::vector<Fractional> terms = compute_fractional_terms(pcd1, pcd2, noise_bound_ * noise_bound_, c_ * c_);

  Eigen::Matrix4d init_mat = compute_initial_guess(pcd1, pcd2);
  x = se3_mat_to_vec(init_mat);

  for (int i = 0; i < max_iteration_; i++) {
    // alternating solve alpha
    alpha = updateAuxiliaryVariables(&terms);
    mat_w = updateWeight(&alpha, &terms);

    // alternating solve x
    x = updateVariable(mat_w);
    update_terms_cache(&terms, &x);

    // stopping criteria
    psi_norm = compute_psi_norm(&alpha, &terms);
    if (psi_norm < tol_) {
      break;
    }
  }

  se3 = se3_vec_to_mat(x);
  se3.block<3, 3>(0, 0) = project(se3.block<3, 3>(0, 0));
  return se3;
}

}  // namespace registration
