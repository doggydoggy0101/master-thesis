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

#include "registration/constant.h"

namespace registration {

namespace outlier_rejection {

/**
 * @brief Turple test in paper: Fast Global Registration.
 *
 * @param src_point_cloud Source point cloud.
 * @param tar_point_cloud Target point cloud.
 *
 * @return Indices of inlier correspondences.
 */
std::vector<int> tuple_test(const PointCloud& src_point_cloud, const PointCloud& tar_point_cloud,
                            const double tuple_scale = 0.95, const int max_tuple_count = 1000);

}  // namespace outlier_rejection

}  // namespace registration