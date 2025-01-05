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

class AbstractSolver {
 protected:
  size_t max_iter;  // Maximum number of iterations.
  double tol;       // Tolerance for early stopping.
  double c;         // Threshold c in the Geman-McClure function.

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
};

class FracGM : public AbstractSolver {
 private:
  Eigen::MatrixXd mat_w;           // weighted quadratic term
  Eigen::Vector<double, 13> temp;  // LU solver
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
  FracGM(const size_t& max_iteration, const double& tolerance, const double& threshold_c);

  /**
   * @brief Auxiliary variables update in the FracGM algorithm.
   *
   * @param terms A vector of fractional terms.
   *
   * @return The auxiliary variables.
   */
  std::vector<double> updateAuxiliaryVariables(std::vector<Fractional>* terms);

  /**
   * @brief Weight update in the FracGM algorithm.
   *
   * @param alpha The auxiliary variables.
   * @param terms A vector of the quadratic terms of square of residuals.
   *
   * @return The weighted quadratic term for the quadratic program.
   */
  Eigen::MatrixXd updateWeight(std::vector<double>* alpha, std::vector<Fractional>* terms);

  /**
   * @brief Variable update in the FracGM algorithm.
   *
   * @param mat_w The weighted quadratic term.
   *
   * @return The closed-form solution of the quadratic program.
   */
  Eigen::VectorXd updateVariable(Eigen::MatrixXd& mat_w);

  /// @brief Compute the norm of psi for stopping criteria.
  float compute_psi_norm(std::vector<double>* alpha, std::vector<Fractional>* terms);

  /// @brief Update the cache in each fractional term.
  void update_terms_cache(std::vector<Fractional>* terms, Eigen::VectorXd* vec);

  /**
   * @brief Solve the point cloud registration problem.
   *
   * @param pcd1 Source point cloud.
   * @param pcd2 Target point cloud.
   * @param noise_bound Noise bound (sigma) of the residual.
   *
   * @return Transformation matrix.
   */
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) override;
};

class QGM : public AbstractSolver {
 private:
  Eigen::MatrixXd mat_w;           // weighted quadratic term
  Eigen::Vector<double, 13> temp;  // LU solver
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
  QGM(const size_t& max_iteration = 1000, const double& tolerance = 1e-6, const double& threshold_c = 0.1);

  /**
   * @brief Weight update in the QGM algorithm.
   *
   * @param terms A vector of the quadratic terms of square of residuals.
   * @param x The variable.
   *
   * @return The weighted quadratic term for the quadratic program.
   */
  Eigen::MatrixXd updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x);

  /**
   * @brief Variable update in the QGM algorithm.
   *
   * @param mat_w The weighted quadratic term.
   *
   * @return The closed-form solution of the quadratic program.
   */
  Eigen::VectorXd updateVariable(Eigen::MatrixXd& mat_w);

  /**
   * @brief Solve the point cloud registration problem.
   *
   * @param pcd1 Source point cloud.
   * @param pcd2 Target point cloud.
   * @param noise_bound Noise bound (sigma) of the residual.
   *
   * @return Transformation matrix.
   */
  Eigen::Matrix4d solve(const PointCloud& pcd1, const PointCloud& pcd2, const double& noise_bound) override;
};

}  // namespace registration
