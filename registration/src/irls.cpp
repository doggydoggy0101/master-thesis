/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/irls.h"

namespace registration {

namespace irls {

std::vector<Eigen::MatrixXd> compute_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2) {
  std::vector<Eigen::MatrixXd> terms;
  terms.reserve(pcd1.rows());

  Eigen::Matrix3d id3 = Eigen::Matrix3d::Identity(3, 3);
  Eigen::MatrixXd mat_n;
  Eigen::MatrixXd mat_m;

  for (int i = 0; i < pcd1.rows(); i++) {
    mat_n = Eigen::Matrix<double, 3, 13>::Zero(3, 13);
    mat_n.block<3, 9>(0, 0) = kroneckerProduct(pcd1.row(i), id3);
    mat_n.block<3, 3>(0, 9) = id3;
    mat_n.block<3, 1>(0, 12) = -pcd2.row(i);

    mat_m = (mat_n.transpose() * mat_n) / (noise_bound_2);
    terms.push_back(mat_m);
  }
  return terms;
}

};  // namespace irls

};  // namespace registration