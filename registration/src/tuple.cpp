// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------

/**
 * Modified by Bang-Shien Chen in 2025 for master-thesis project.
 */

#include "registration/tuple.h"

std::vector<int> registration::outlier_rejection::tuple_test(const PointCloud& src_point_cloud,
                                                             const PointCloud& tar_point_cloud, double tuple_scale,
                                                             int maximum_tuple_count) {
  int rand0, rand1, rand2, cnt = 0;
  int ncorr = static_cast<int>(src_point_cloud.rows());
  int number_of_trial = ncorr * 100;

  // random generator
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<> rand_generator(0, ncorr - 1);

  std::vector<int> corres_tuple;
  for (int i = 0; i < number_of_trial; i++) {
    rand0 = rand_generator(gen);
    rand1 = rand_generator(gen);
    rand2 = rand_generator(gen);

    // collect 3 points from source fragment
    Eigen::Vector3d pti0 = src_point_cloud.row(rand0);
    Eigen::Vector3d pti1 = src_point_cloud.row(rand1);
    Eigen::Vector3d pti2 = src_point_cloud.row(rand2);
    double li0 = (pti0 - pti1).norm();
    double li1 = (pti1 - pti2).norm();
    double li2 = (pti2 - pti0).norm();

    // collect 3 points from target fragment
    Eigen::Vector3d ptj0 = tar_point_cloud.row(rand0);
    Eigen::Vector3d ptj1 = tar_point_cloud.row(rand1);
    Eigen::Vector3d ptj2 = tar_point_cloud.row(rand2);
    double lj0 = (ptj0 - ptj1).norm();
    double lj1 = (ptj1 - ptj2).norm();
    double lj2 = (ptj2 - ptj0).norm();

    // check tuple constraint
    if ((li0 * tuple_scale < lj0) && (lj0 < li0 / tuple_scale) && (li1 * tuple_scale < lj1) &&
        (lj1 < li1 / tuple_scale) && (li2 * tuple_scale < lj2) && (lj2 < li2 / tuple_scale)) {
      corres_tuple.push_back(rand0);
      corres_tuple.push_back(rand1);
      corres_tuple.push_back(rand2);
      cnt++;
    }
    if (cnt >= maximum_tuple_count) break;
  }
  return corres_tuple;
}
