/**
 * Copyright 2020, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Jingnan Shi, et al. (see THANKS for the full author list)
 *
 * MIT License
 */

/**
 * Modified by Yu-Kai Lin in 2024 for QGM project.
 */
#pragma once

#include <Eigen/Dense>
#include <vector>

#include "omp.h"
#include "pmc/pmc.h"
#include "registration/graph.h"

namespace registration {

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3>;

namespace mcis {

/**
 * This function takes two point clouds and their noise bound, and timeout in
 * seconds for the max clique solver, and returns the indices of inlier
 * correspondences.
 *
 * This function invokes the max clique inlier selection (MCIS) algorithm
 * coming from the TEASER++ solver.
 *
 * The max clique solver is run with the number of threads equal to the number
 * of logical CPU cores available.
 *
 * # Returns
 * Indices of inlier correspondences.
 */
std::vector<int> inlier_selection(const PointCloud& src, const PointCloud& dst, double noise_bound, double pmc_timeout,
                                  int pmc_n_threads);

}  // namespace mcis

}  // namespace registration