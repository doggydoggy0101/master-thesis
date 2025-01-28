/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <vector>
#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>

#include "registration/constant.h"

namespace registration {

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

namespace fracgm {

// A structure that can be used to compute the quadratic form associated
// with a matrix, and keep track of the most recently computed value.
struct R2Sym {
 public:
  Eigen::MatrixXd mat;
  double cache;

  R2Sym();  // default
  R2Sym(Eigen::MatrixXd);
  R2Sym(Eigen::MatrixXd, double);

  // Compute the quadratic form associated with self and x.
  double call(Eigen::VectorXd);

  // Update the cached value of the quadratic form associated with self.
  void update_cache(Eigen::VectorXd);
};

// A structure to represent a fractional term $f(x)/h(x)$ in the
// Geman-McClure-based objective function.
struct Fractional {
 public:
  R2Sym r2;
  double c2;

  Fractional(R2Sym, double);

  // Updates the cache of the square of residual.
  void update_cache(Eigen::VectorXd);

  // Computes the numerator $f(x)$.
  double f();
  // Computes the denominator $h(x)$.
  double h();
  // Get the matrix associated with the numerator.
  Eigen::MatrixXd f_mat();
  // Get the matrix associated with the denominator.
  Eigen::MatrixXd h_mat();
};

/**
 * @brief Compute the fractional terms.
 *
 * @param pcd1`          - The source point cloud.
 * @param pcd2`          - The target point cloud.
 * @param noise_bound_2` - The square of noise bound.
 * @param c2`            - The square of threshold c.
 *
 * @return A vector of quadratic matrices.
 */
std::vector<Fractional> compute_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2, double c2);

/// @brief Compute the norm of psi for stopping criteria.
float compute_psi_norm(std::vector<double>* alpha, std::vector<Fractional>* terms);

/// @brief Update the cache in each fractional term.
void update_terms_cache(std::vector<Fractional>* terms, Eigen::VectorXd* vec);

};  // namespace fracgm

};  // namespace registration