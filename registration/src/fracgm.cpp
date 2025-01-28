/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/fracgm.h"

namespace registration {

namespace fracgm {

// R2Sym
R2Sym::R2Sym() {
  mat = Eigen::MatrixXd::Zero(13, 13);
  cache = 0.0;
}

R2Sym::R2Sym(Eigen::MatrixXd mat_) {
  mat = mat_;
  cache = 0.0;
}

R2Sym::R2Sym(Eigen::MatrixXd mat_, double cache_) {
  mat = mat_;
  cache = cache_;
}

double R2Sym::call(Eigen::VectorXd x) { return x.transpose() * mat * x; }

void R2Sym::update_cache(Eigen::VectorXd x) { cache = call(x); }

// Fractional
Fractional::Fractional(R2Sym r2_, double c2_) {
  r2 = r2_;
  c2 = c2_;
}

void Fractional::update_cache(Eigen::VectorXd x) { r2.update_cache(x); }

double Fractional::f() { return c2 * r2.cache; }
double Fractional::h() { return r2.cache + c2; }

Eigen::MatrixXd Fractional::f_mat() { return c2 * r2.mat; }
Eigen::MatrixXd Fractional::h_mat() { return r2.mat; }

std::vector<Fractional> compute_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2, double c2) {
  std::vector<Fractional> terms;
  terms.reserve(pcd1.rows());
  Eigen::MatrixXd mat_n;
  Eigen::MatrixXd mat_m;

  for (int i = 0; i < pcd1.rows(); i++) {
    mat_n = Eigen::Matrix<double, 3, 13>::Zero(3, 13);
    mat_n.block<3, 9>(0, 0) = kroneckerProduct(pcd1.row(i), eye3);
    mat_n.block<3, 3>(0, 9) = eye3;
    mat_n.block<3, 1>(0, 12) = -pcd2.row(i);

    mat_m = (mat_n.transpose() * mat_n) / (noise_bound_2);
    terms.push_back(Fractional(R2Sym(mat_m), c2));
  }
  return terms;
}

float compute_psi_norm(std::vector<double>* alpha, std::vector<Fractional>* terms) {
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

void update_terms_cache(std::vector<Fractional>* terms, Eigen::VectorXd* vec) {
  for (auto& term : *terms) {
    term.update_cache(*vec);
  }
}

}; // namespace fracgm


};  // namespace registration