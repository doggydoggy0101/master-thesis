// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

/**
 * Modified by Bang-Shien Chen in 2025 for master-thesis project.
 */
#pragma once

#include <Eigen/Dense>
#include <random>
#include <vector>

namespace registration {

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

namespace outlier_rejection {

/**
 * @brief Turple test in paper: Fast Global Registration.
 *
 * @param src_point_cloud Source point cloud.
 * @param tar_point_cloud Target point cloud.
 *
 * @return Indices of inlier correspondences.
 */
std::vector<int> tuple_test(const PointCloud& src_point_cloud, const PointCloud& tar_point_cloud, double& tuple_scale,
                            int& max_tuple_count);

}  // namespace outlier_rejection

}  // namespace registration