/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace registration {

/**
 * @brief Project a 3 by 3 matrix onto the nearest rotation matrix.
 *
 * @param mat The matrix to be projected.
 *
 * @return The rotation matrix.
 */
Eigen::Matrix3d project(const Eigen::Matrix3d mat);

/**
 * @brief Reshape matrix to vector.
 *
 * @param mat` - The matrix to be reshaped.
 *
 * @return The reshaped vector.
 */
Eigen::VectorXd se3_mat_to_vec(Eigen::Matrix4d mat);

/**
 * @brief Reshape vector to matrix.
 *
 * @param vec` - The vector to be reshaped.
 *
 * @return The reshaped matrix.
 */
Eigen::Matrix4d se3_vec_to_mat(Eigen::VectorXd vec);

};  // namespace registration