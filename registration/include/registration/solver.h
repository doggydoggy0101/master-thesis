/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>
#include <vector>

#include "registration/irls.h"
#include "registration/fracgm.h"
#include "registration/utils.h"

namespace registration {

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

class AbstractSolver {
 protected:
  size_t max_iter; // Maximum number of iterations.
  double tol;      // Tolerance for early stopping.

 private:
  Eigen::Vector<double, 13> temp; // Quadratic program.

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
  Eigen::VectorXd solveQuadraticProgram(Eigen::MatrixXd& mat){
    if (mat.fullPivLu().rank() == mat.rows()) {
      // solve by closed-form
      temp = mat.ldlt().solve(e);
    } else {
      // solve by pseudo-inverse
      temp = mat.completeOrthogonalDecomposition().solve(e);
    }
    return (1 / (e.transpose() * temp)) * temp;
  }

  /// @brief Check convergence of relative cost difference.
  bool checkCostConvergence(double& prev_cost, double& curr_cost) {
    return (std::abs(curr_cost - prev_cost) / std::max(prev_cost, 1e-7)) < this->tol;
  }
};

class IrlsSolver : public AbstractSolver {
 protected:
  double c;         // Threshold c in the Geman-McClure function.

 private:
  Eigen::MatrixXd mat_w;           // weighted quadratic term
  Eigen::VectorXd x;               // solution vector
  Eigen::Matrix4d se3;             // solution matrix

 public:
  /**
   * @brief QGM-based registration solver with linear relaxation.
   *
   * @param max_iteration The maximum number of iterations allowed.
   * @param tolerance The tolerance for convergence.
   * @param threshold_c The parameter $c$ of the Geman-McClure function.
   */
  IrlsSolver(const size_t& max_iteration = 1000, const double& tolerance = 1e-6, const double& threshold_c = 0.1);

  /**
   * @brief Weight update in the QGM algorithm.
   *
   * @param terms A vector of the quadratic terms of square of residuals.
   * @param x The variable.
   *
   * @return The weighted quadratic term for the quadratic program.
   */
  Eigen::MatrixXd updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x);

  /// @brief Solve the point cloud registration problem.
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) override;
};

class FracgmSolver : public AbstractSolver {
 protected:
  double c;         // Threshold c in the Geman-McClure function.

 private:
  Eigen::MatrixXd mat_w;           // weighted quadratic term
  Eigen::VectorXd x;               // solution vector
  Eigen::Matrix4d se3;             // solution matrix
  std::vector<double> alpha;       // auxiliary variables
  float psi_norm;                  // stopping criteria

 public:
  /**
   * @brief FracGM-based registration solver with linear relaxation.
   *
   * @param max_iteration The maximum number of iterations allowed.
   * @param tolerance The tolerance for convergence.
   * @param threshold_c The parameter $c$ of the Geman-McClure function.
   */
  FracgmSolver(const size_t& max_iteration, const double& tolerance, const double& threshold_c);

  /**
   * @brief Auxiliary variables update in the FracGM algorithm.
   *
   * @param terms A vector of fractional terms.
   *
   * @return The auxiliary variables.
   */
  std::vector<double> updateAuxiliaryVariables(std::vector<fracgm::Fractional>* terms);

  /**
   * @brief Weight update in the FracGM algorithm.
   *
   * @param alpha The auxiliary variables.
   * @param terms A vector of the quadratic terms of square of residuals.
   *
   * @return The weighted quadratic term for the quadratic program.
   */
  Eigen::MatrixXd updateWeight(std::vector<double>* alpha, std::vector<fracgm::Fractional>* terms);

  /// @brief Solve the point cloud registration problem.
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) override;
};

}  // namespace registration
