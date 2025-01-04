/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <unsupported/Eigen/KroneckerProduct>
#include <vector>

namespace registration {

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

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
 * Get the zero mean of a point cloud.
 *
 * # Arguments
 *
 * - `pcd` - The point cloud to be computed.
 *
 * # Returns
 *
 * - Turples of the zero mean point cloud and the mean vector.
 */
std::pair<PointCloud, Eigen::Vector3d> get_zero_mean_point_cloud(const PointCloud pcd);

/**
 * Project a 3 by 3 matrix onto the nearest rotation matrix.
 *
 * # Arguments
 *
 * - `mat` - The matrix to be projected.
 *
 * # Returns
 *
 * - The rotation matrix.
 */
Eigen::Matrix3d project(const Eigen::Matrix3d mat);

/**
 * Horn's closed-form solution for point cloud registration.
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
Eigen::Matrix4d compute_initial_guess(PointCloud pcd1, PointCloud pcd2);

/**
 * Compute the quadratic matrix of the square of residuals.
 *
 * # Arguments
 *
 * - `pcd1`          - The source point cloud.
 * - `pcd2`          - The target point cloud.
 * - `noise_bound_2` - The square of noise bound.
 *
 * # Returns
 *
 * - A vector of quadratic matrices.
 */
std::vector<Eigen::MatrixXd> compute_residual_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2);

/**
 * Compute the fractional terms.
 *
 * # Arguments
 *
 * - `pcd1`          - The source point cloud.
 * - `pcd2`          - The target point cloud.
 * - `noise_bound_2` - The square of noise bound.
 * - `c2`            - The square of threshold c.
 *
 * # Returns
 *
 * - A vector of quadratic matrices.
 */
std::vector<Fractional> compute_fractional_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2, double c2);

/**
 * Reshape matrix to vector.
 *
 * # Arguments
 *
 * - `mat` - The matrix to be reshaped.
 *
 * # Returns
 *
 * - The reshaped vector.
 */
Eigen::VectorXd se3_mat_to_vec(Eigen::Matrix4d mat);

/**
 * Reshape vector to matrix.
 *
 * # Arguments
 *
 * - `vec` - The vector to be reshaped.
 *
 * # Returns
 *
 * - The reshaped matrix.
 */
Eigen::Matrix4d se3_vec_to_mat(Eigen::VectorXd vec);

};  // namespace registration