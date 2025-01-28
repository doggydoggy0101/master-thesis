/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/KroneckerProduct>

#include "registration/constant.h"

namespace registration {

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

namespace irls {

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

};  // namespace irls

};  // namespace registration