/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "registration/constant.h"
#include "registration/fracgm.h"
#include "registration/gnc.h"
#include "registration/irls.h"
#include "registration/utils.h"

namespace registration {

class AbstractSolver {
 protected:
  size_t max_iter;     // Maximum number of iterations.
  double tol;          // Tolerance for early stopping.
  std::string robust;  // Robust function.
  double c;            // Threshold c.

 private:
  Eigen::Vector<double, 13> temp;  // Quadratic program.

 public:
  virtual ~AbstractSolver() {}

  /**
   * @brief Pure virtual method for solving the point cloud registration problem.
   *
   * @param pcd1 Source point cloud.
   * @param pcd2 Target point cloud.
   * @param noise_bound Noise bound (sigma) of the residual.
   *
   * @return Transformation matrix.
   */
  virtual Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) = 0;

  /**
   * @brief Closed-form for the quadratic program:
   *
   *            min  x.T @ mat @ x
   *            s.t. e.T @ x = 1.
   *
   *        This has a closed-form solution:
   *
   *            x = (inv(mat) @ e) / (e.T @ inv(mat) @ e).
   *
   *        NOTE: This requires mat to be invertible/singular/full-rank.
   *        We use the pseudo-inverse if the matrix is non-singular.
   *
   * @param mat The quadratic term.
   *
   * @return Solution of the quadratic program.
   */
  Eigen::VectorXd solveQuadraticProgram(Eigen::MatrixXd& mat) {
    if (mat.fullPivLu().rank() == mat.rows()) {
      // solve by closed-form
      temp = mat.ldlt().solve(e13);
    } else {
      // solve by pseudo-inverse
      temp = mat.completeOrthogonalDecomposition().solve(e13);
    }
    return (1 / (e13.transpose() * temp)) * temp;
  }

  /// @brief Check convergence of relative cost difference.
  bool checkCostConvergence(double& prev_cost, double& curr_cost) {
    return (std::abs(curr_cost - prev_cost) / std::max(prev_cost, 1e-7)) < this->tol;
  }
};

class IrlsSolver : public AbstractSolver {
 private:
  Eigen::MatrixXd mat_w;  // weighted quadratic term
  Eigen::VectorXd vec_w;  // weight vector
  Eigen::VectorXd x;      // solution vector
  Eigen::Matrix4d se3;    // solution matrix

 public:
  /**
   * @brief IRLS-based registration solver with linear relaxation.
   *
   * @param max_iteration The maximum number of iterations allowed.
   * @param tolerance The tolerance for convergence.
   * @param robust_type Robust function: Truncated Least Squares (TLS), Geman-McClure (GM).
   * @param threshold_c The parameter $c$ of the robust function.
   */
  IrlsSolver(const size_t& max_iteration = 1000, const double& tolerance = 1e-6, const std::string& robust_type = "GM",
             const double& threshold_c = 1.0);

  /// @brief Solve the point cloud registration problem.
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound = 0.1) override;
};

class GncSolver : public AbstractSolver {
 protected:
  double gnc_factor_;  // surrogate parameter's update step size
  double weight_tol;   // stopping critera for TLS weight
  bool major;          // majorization surrogate
  bool superlinear_;   // superlinear surrogate parameter

 private:
  Eigen::MatrixXd mat_w;  // weighted quadratic term
  Eigen::VectorXd vec_w;  // weight vector
  Eigen::VectorXd x;      // solution vector
  Eigen::Matrix4d se3;    // solution matrix
  double mu;              // surrogate paramter

 public:
  /**
   * @brief GNC-based registration solver with linear relaxation.
   *
   * @param max_iteration The maximum number of iterations allowed.
   * @param tolerance The tolerance for convergence.
   * @param robust_type Robust function: Truncated Least Squares (TLS), Geman-McClure (GM) or L0-norm (L0).
   * @param threshold_c The parameter $c$ of the robust function.
   * @param gnc_factor Surrogate parameter's update step size.
   * @param weight_tolerance Stopping critera for weights being binary.
   * @param majorization Use majorization surrogate for TLS (does nothing for other robust function).
   * @param superlinear Use superlienar update surrogate parameter for TLS (does nothing for other robust function).
   */
  GncSolver(const size_t& max_iteration = 1000, const double& tolerance = 1e-6, const std::string& robust_type = "GM",
            const double& threshold_c = 1.0, const double& gnc_factor = 1.4, const double& weight_tolerance = 1e-4,
            const bool& majorization = false, const bool& superlinear = false);

  /// @brief Solve the point cloud registration problem.
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound = 0.1) override;
};

class FracgmSolver : public AbstractSolver {
 private:
  Eigen::MatrixXd mat_w;      // weighted quadratic term
  Eigen::VectorXd x;          // solution vector
  Eigen::Matrix4d se3;        // solution matrix
  std::vector<double> alpha;  // auxiliary variables
  float psi_norm;             // stopping criteria

 public:
  /**
   * @brief FracGM-based registration solver with linear relaxation.
   *
   * @param max_iteration The maximum number of iterations allowed.
   * @param tolerance The tolerance for convergence.
   * @param threshold_c The parameter $c$ of the Geman-McClure function.
   */
  FracgmSolver(const size_t& max_iteration = 1000, const double& tolerance = 1e-6, const double& threshold_c = 1.0);

  /// @brief Solve the point cloud registration problem.
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound = 0.1) override;
};

}  // namespace registration
