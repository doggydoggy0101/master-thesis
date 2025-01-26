/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/utils.h"

namespace registration {

// R2Sym
R2Sym::R2Sym() {
  mat = Eigen::MatrixXd::Zero(13, 13);
  cache = 0.0;
}

R2Sym::R2Sym(Eigen::MatrixXd mat_) {
  mat = mat_;
  cache = 0.0;
}

R2Sym::R2Sym(Eigen::MatrixXd mat_, double cache_) {
  mat = mat_;
  cache = cache_;
}

double R2Sym::call(Eigen::VectorXd x) { return x.transpose() * mat * x; }

void R2Sym::update_cache(Eigen::VectorXd x) { cache = call(x); }

// Fractional
Fractional::Fractional(R2Sym r2_, double c2_) {
  r2 = r2_;
  c2 = c2_;
}

void Fractional::update_cache(Eigen::VectorXd x) { r2.update_cache(x); }

double Fractional::f() { return c2 * r2.cache; }
double Fractional::h() { return r2.cache + c2; }

Eigen::MatrixXd Fractional::f_mat() { return c2 * r2.mat; }
Eigen::MatrixXd Fractional::h_mat() { return r2.mat; }

std::pair<PointCloud, Eigen::Vector3d> get_zero_mean_point_cloud(PointCloud pcd) {
  Eigen::Vector3d mean = pcd.colwise().mean();

  for (int i = 0; i < pcd.rows(); i++) {
    pcd.row(i) -= mean.transpose();
  }

  return std::make_pair(pcd, mean);
}

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

std::vector<Eigen::MatrixXd> compute_residual_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2) {
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

std::vector<Fractional> compute_fractional_terms(PointCloud pcd1, PointCloud pcd2, double noise_bound_2, double c2) {
  std::vector<Fractional> terms;
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
    terms.push_back(Fractional(R2Sym(mat_m), c2));
  }
  return terms;
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