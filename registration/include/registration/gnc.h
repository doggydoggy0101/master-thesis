/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <string>
#include <unsupported/Eigen/KroneckerProduct>
#include <utility>

#include "registration/constant.h"

namespace registration {

namespace gnc {

/**
 * Compute the quadratic matrix of the square of residuals.
 *
 * # Arguments
 *
 * @param pcd1 The source point cloud.
 * @param pcd2 The target point cloud.
 * @param noise_bound_2 The square of noise bound.
 *
 * @return A vector of quadratic matrices.
 */
std::vector<Eigen::MatrixXd> compute_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2);

/**
 * @brief Compute the initial value for surrogate paramter mu.
 *
 * @param res_sq Maximum square of residuals.
 * @param robust Robust function type.
 * @param robust Robust function type.
 * @param c Parameter in robust function.
 *
 * @return Initial guess of mu.
 */
double compute_initial_mu(double& res_sq, std::string& robust, double& c);

/**
 * @brief Update sorrogate paramter mu.
 *
 * @param mu Sorrogate paramerer.
 * @param robust Robust function type.
 * @param gnc_factor Update step size.
 */
void update_mu(double& mu, std::string& robust, double& gnc_factor);

/**
 * @brief Weight update in the GNC algorithm.
 *
 * @param terms A vector of the quadratic terms of square of residuals.
 * @param x The variable.
 * @param robust Robust function type.
 * @param c Parameter in robust function.
 * @param mu Sorrogate parameter.
 *
 * @return The weighted quadratic term for the quadratic program.
 * @return The weight vector.
 */
std::pair<Eigen::MatrixXd, Eigen::VectorXd> updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x,
                                                         std::string& robust, double& c, double& mu);

/// @brief Check if surrogate function approximates the original robust function (for GM).
bool check_mu_convergence(double& mu, std::string& robust);

/// @brief Check convergence of weights to binary values (for TLS).
bool check_weight_convergence(Eigen::VectorXd& weight, std::string& robust, double& weight_tol);

};  // namespace gnc

};  // namespace registration