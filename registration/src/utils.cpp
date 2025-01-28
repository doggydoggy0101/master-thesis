/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/utils.h"

namespace registration {

Eigen::Matrix3d project(const Eigen::Matrix3d mat) {
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(mat, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  Eigen::Matrix3d rot = U * V.transpose();

  if (rot.determinant() < 0) {
    Eigen::Vector3d d(1.0, 1.0, -1.0);
    rot = U * d.asDiagonal() * V.transpose();
  }

  return rot;
}

Eigen::VectorXd se3_mat_to_vec(Eigen::Matrix4d mat) {
  Eigen::Vector<double, 13> vec;
  vec << mat(0, 0), mat(1, 0), mat(2, 0), mat(0, 1), mat(1, 1), mat(2, 1), mat(0, 2), mat(1, 2), mat(2, 2), mat(0, 3),
      mat(1, 3), mat(2, 3), 1.0;
  return vec;
}

Eigen::Matrix4d se3_vec_to_mat(Eigen::VectorXd vec) {
  Eigen::Matrix4d mat;
  mat << vec(0), vec(3), vec(6), vec(9), vec(1), vec(4), vec(7), vec(10), vec(2), vec(5), vec(8), vec(11), 0.0, 0.0,
      0.0, 1.0;
  return mat;
}

};  // namespace registration