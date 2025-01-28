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

std::pair<Eigen::MatrixXd, Eigen::VectorXd> updateWeight(std::vector<Eigen::MatrixXd>& terms, Eigen::VectorXd& x,
                                                         std::string& robust, double& c) {
  Eigen::MatrixXd mat_w = Eigen::MatrixXd::Zero(13, 13);
  Eigen::VectorXd vec_w = Eigen::VectorXd::Zero(terms.size());

  if (robust == "TLS") {
    for (size_t i = 0; i < terms.size(); i++) {
      if (x.transpose() * terms[i] * x <= c * c) {
        mat_w += terms[i];
        vec_w[i] = 1.0;
      }
    }
  } else if (robust == "GM") {
    for (size_t i = 0; i < terms.size(); i++) {
      double w_i = 1.0 / ((x.transpose() * terms[i] * x + c * c) * (x.transpose() * terms[i] * x + c * c));
      mat_w += w_i * terms[i];
      vec_w[i] = w_i;
    }
  }
  return std::make_pair(mat_w, vec_w);
}

};  // namespace irls

};  // namespace registration