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

class qgm {
 private:
  size_t max_iteration_;
  double tol_;
  double c_;
  double noise_bound_;

  Eigen::MatrixXd mat_w;           // weighted quadratic term
  Eigen::Vector<double, 13> temp;  // LU solver
  Eigen::VectorXd x;               // solution vector
  Eigen::Matrix4d se3;             // solution matrix

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
  qgm(const size_t& max_iteration, const double& tol, const double& c, const double& noise_bound);

  /**
   * Weight update in the QGM algorithm.
   *
   * # Arguments
   *
   * - `terms` - A vector of the quadratic terms of square of residuals.
   * - `x`     - The variable.
   *
   * # Returns
   *
   * The weighted quadratic term for the quadratic program.
   */
  Eigen::MatrixXd updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x);

  /**
   * Variable update in the QGM algorithm.
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