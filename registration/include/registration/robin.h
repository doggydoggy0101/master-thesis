/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <algorithm>
#include <fstream>

#include "registration/constant.h"
#include "robin/robin.hpp"

namespace registration {

namespace outlier_rejection {

/**
 * @brief ROBIN: a Graph-Theoretic Approach to Reject Outliers in Robust Estimation using Invariants.
 *
 * @param src_point_cloud Source point cloud.
 * @param tar_point_cloud Target point cloud.
 * @param robin_mode ROBIN's graph type: "max_core" and "max_clique".
 *
 * @return Indices of inlier correspondences.
 */
std::vector<size_t> robin(const PointCloud& src_point_cloud, const PointCloud& tar_point_cloud,
                          const double noise_bound, const std::string robin_mode);

}  // namespace outlier_rejection

}  // namespace registration