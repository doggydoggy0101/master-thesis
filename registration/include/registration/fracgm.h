/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "registration/constant.h"
#include "registration/utils.h"

namespace registration {

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

class fracgm {
 private:
  size_t max_iteration_;
  double tol_;
  double c_;
  double noise_bound_;

  Eigen::MatrixXd mat_w;           // weighted quadratic term
  Eigen::Vector<double, 13> temp;  // LU solver
  Eigen::VectorXd x;               // solution vector
  Eigen::Matrix4d se3;             // solution matrix
  std::vector<double> alpha;       // auxiliary variables
  float psi_norm;                  // stopping criteria

 public:
  /**
   * QGM-based registration solver with linear (naive) relaxation.
   *
   * # Arguments
   *
   * - `max_iteration` - The maximum number of iterations allowed.
   * - `tol`           - The tolerance for convergence.
   * - `c`             - The parameter $c$ of the Geman-McClure function.
   * - `noise_bound`   - The noise bound $\sigma$ for the residual function.
   */
  fracgm(const size_t& max_iteration, const double& tol, const double& c, const double& noise_bound);

  /**
   * Auxiliary variables update in the FracGM algorithm.
   *
   * # Arguments
   *
   * - `terms` - A vector of fractional terms.
   *
   * # Returns
   *
   * - The auxiliary variables.
   */
  std::vector<double> updateAuxiliaryVariables(std::vector<Fractional>* terms);

  /**
   * Weight update in the FracGM algorithm.
   *
   * # Arguments
   *
   * - `alpha` - The auxiliary variables.
   * - `terms` - A vector of fractional terms.
   *
   * # Returns
   *
   * The weighted quadratic term for the quadratic program.
   */
  Eigen::MatrixXd updateWeight(std::vector<double>* alpha, std::vector<Fractional>* terms);

  /**
   * Variable update in the FracGM algorithm.
   *
   * # Arguments
   *
   * - `mat_w` - The weighted quadratic term.
   *
   * # Returns
   *
   * The closed-form solution of the quadratic program.
   */
  Eigen::VectorXd updateVariable(Eigen::MatrixXd& mat_w);

  /// Compute the norm of psi for stopping criteria.
  float compute_psi_norm(std::vector<double>* alpha, std::vector<Fractional>* terms);

  /// Update the cache in each fractional term.
  void update_terms_cache(std::vector<Fractional>* terms, Eigen::VectorXd* vec);

  /**
   * Solve the point cloud registration problem.
   *
   * # Arguments
   *
   * - `pcd1` - The source point cloud.
   * - `pcd2` - The target point cloud.
   *
   * # Returns
   *
   * The estimated registration.
   */
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2);
};

};  // namespace registration