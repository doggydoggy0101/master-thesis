/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include <Eigen/Dense>
#include <fstream>
#include <iostream>
#include <sstream>
#include <tuple>
#include <vector>

#include "registration/mcis.h"  // maximum clique inlier selection
#include "registration/parameter.h"
#include "registration/robin.h"  // robin graph prune
#include "registration/solver.h"
#include "registration/tuple.h"  // tuple test

using PointCloud = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> read_matrix(std::string filename) {
  std::ifstream file(filename);
  std::string line;

  std::vector<std::vector<double>> data;
  std::vector<double> line_data;

  int n_cols = 0;
  while (std::getline(file, line)) {
    std::stringstream ss(line);
    int _n_cols = 0;

    line_data.clear();
    double value;
    while (ss >> value) {
      line_data.push_back(value);
      _n_cols++;
    }

    if (n_cols == 0) {
      n_cols = _n_cols;
    } else if (n_cols != _n_cols) {
      throw std::runtime_error("Invalid number of columns in file");
    }

    data.push_back(line_data);
  }

  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> matrix(data.size(), n_cols);

  for (int i = 0; i < (int)data.size(); i++) {
    for (int j = 0; j < n_cols; j++) {
      matrix(i, j) = data[i][j];
    }
  }

  return matrix;
}

std::tuple<PointCloud, PointCloud, Eigen::Matrix<double, 4, 4, Eigen::RowMajor>> get_toy_data() {
  PointCloud src = read_matrix("../../data/cloud_bin_0.txt");
  PointCloud dst = read_matrix("../../data/cloud_bin_1.txt");

  Eigen::Matrix<double, 4, 4, Eigen::RowMajor> gt = Eigen::Matrix4d::Identity();
  gt.block<4, 4>(0, 0) = read_matrix("../../data/gt.txt");

  return std::make_tuple(src, dst, gt);
}

std::tuple<PointCloud, PointCloud> perform_tuple(const PointCloud &pc1, const PointCloud &pc2, double tuple_scale,
                                                 int max_tuple_count) {
  auto indices = registration::outlier_rejection::tuple_test(pc1, pc2, tuple_scale, max_tuple_count);

  if (indices.empty()) return std::make_tuple(pc1, pc2);

  PointCloud inlier_pc1(indices.size(), 3);
  PointCloud inlier_pc2(indices.size(), 3);

  for (size_t idx = 0; idx < indices.size(); idx++) {
    auto index = indices[idx];
    inlier_pc1.row(idx) = pc1.row(index);
    inlier_pc2.row(idx) = pc2.row(index);
  }

  return std::make_tuple(inlier_pc1, inlier_pc2);
}

std::tuple<PointCloud, PointCloud> perform_mcis(const PointCloud &pc1, const PointCloud &pc2, double noise_bound,
                                                double pmc_timeout, int pmc_n_threads) {
  auto indices = registration::outlier_rejection::maximum_clique_inlier_selection(pc1, pc2, noise_bound, pmc_timeout,
                                                                                  pmc_n_threads);

  if (indices.empty()) return std::make_tuple(pc1, pc2);

  PointCloud inlier_pc1(indices.size(), 3);
  PointCloud inlier_pc2(indices.size(), 3);

  for (size_t idx = 0; idx < indices.size(); idx++) {
    auto index = indices[idx];
    inlier_pc1.row(idx) = pc1.row(index);
    inlier_pc2.row(idx) = pc2.row(index);
  }

  return std::make_tuple(inlier_pc1, inlier_pc2);
}

std::tuple<PointCloud, PointCloud> perform_robin(const PointCloud &pc1, const PointCloud &pc2, double noise_bound,
                                                 std::string robin_mode) {
  auto indices = registration::outlier_rejection::robin(pc1, pc2, noise_bound, robin_mode);

  if (indices.empty()) return std::make_tuple(pc1, pc2);

  PointCloud inlier_pc1(indices.size(), 3);
  PointCloud inlier_pc2(indices.size(), 3);

  for (size_t idx = 0; idx < indices.size(); idx++) {
    auto index = indices[idx];
    inlier_pc1.row(idx) = pc1.row(index);
    inlier_pc2.row(idx) = pc2.row(index);
  }

  return std::make_tuple(inlier_pc1, inlier_pc2);
}

int main() {
  // data assumption
  double noise_bound = 0.1;
  // outlier rejection method
  std::string method = "robin";  // "tuple", "mcis", "robin"
  // tuple
  double tuple_scale = 0.95;
  int max_tuple_count = 1000;
  // mcis
  double pmc_timeout = 3600.0;
  int pmc_n_threads = 4;
  // robin
  std::string robin_mode = "max_core";  // "max_core", "max_clique"

  auto [src_reg, dst_reg, gt_reg] = get_toy_data();
  std::cout << "Ground Truth:" << "\n" << gt_reg << "\n\n";

  PointCloud inlier_src_reg, inlier_dst_reg;

  if (method == "tuple") {
    std::tie(inlier_src_reg, inlier_dst_reg) = perform_tuple(src_reg, dst_reg, tuple_scale, max_tuple_count);
  } else if (method == "mcis") {
    std::tie(inlier_src_reg, inlier_dst_reg) = perform_mcis(src_reg, dst_reg, noise_bound, pmc_timeout, pmc_n_threads);
  } else if (method == "robin") {
    std::tie(inlier_src_reg, inlier_dst_reg) = perform_robin(src_reg, dst_reg, noise_bound, robin_mode);
  } else {
    inlier_src_reg = src_reg;
    inlier_dst_reg = dst_reg;
  }

  registration::Params irls_tls_params;
  irls_tls_params.max_iteration = 100;
  irls_tls_params.tolerance = 1e-6;
  irls_tls_params.robust_type = "TLS";
  irls_tls_params.threshold_c = 1.0;
  auto irls_tls_reg = registration::IrlsSolver(irls_tls_params).solve(inlier_src_reg, inlier_dst_reg, noise_bound);
  std::cout << "IRLS-TLS:" << "\n" << irls_tls_reg << "\n\n";

  registration::Params irls_gm_params;
  irls_gm_params.max_iteration = 100;
  irls_gm_params.tolerance = 1e-6;
  irls_gm_params.robust_type = "GM";
  irls_gm_params.threshold_c = 1.0;
  auto irls_gm_reg = registration::IrlsSolver(irls_gm_params).solve(inlier_src_reg, inlier_dst_reg, noise_bound);
  std::cout << "IRLS-GM:" << "\n" << irls_gm_reg << "\n\n";

  registration::Params gnc_tls_params;
  gnc_tls_params.max_iteration = 100;
  gnc_tls_params.tolerance = 1e-6;
  gnc_tls_params.robust_type = "TLS";
  gnc_tls_params.threshold_c = 1.0;
  auto gnc_tls_reg = registration::GncSolver(gnc_tls_params).solve(inlier_src_reg, inlier_dst_reg, noise_bound);
  std::cout << "GNC-TLS:" << "\n" << gnc_tls_reg << "\n\n";

  registration::Params gnc_gm_params;
  gnc_gm_params.max_iteration = 100;
  gnc_gm_params.tolerance = 1e-6;
  gnc_gm_params.robust_type = "GM";
  gnc_gm_params.threshold_c = 1.0;
  auto gnc_gm_reg = registration::GncSolver(gnc_gm_params).solve(inlier_src_reg, inlier_dst_reg, noise_bound);
  std::cout << "GNC-GM:" << "\n" << gnc_gm_reg << "\n\n";

  registration::Params ms_gnc_l0_params;
  ms_gnc_l0_params.max_iteration = 100;
  ms_gnc_l0_params.tolerance = 1e-6;
  ms_gnc_l0_params.robust_type = "L0";
  ms_gnc_l0_params.threshold_c = 1.0;
  auto ms_gnc_l0_reg = registration::GncSolver(ms_gnc_l0_params).solve(inlier_src_reg, inlier_dst_reg, noise_bound);
  std::cout << "MS-GNC-L0:" << "\n" << ms_gnc_l0_reg << "\n\n";

  registration::Params ms_gnc_tls_params;
  ms_gnc_tls_params.max_iteration = 100;
  ms_gnc_tls_params.tolerance = 1e-6;
  ms_gnc_tls_params.robust_type = "TLS";
  ms_gnc_tls_params.threshold_c = 1.0;
  ms_gnc_tls_params.majorization = true;
  ms_gnc_tls_params.superlinear = true;
  auto ms_gnc_tls_reg = registration::GncSolver(ms_gnc_tls_params).solve(inlier_src_reg, inlier_dst_reg, noise_bound);
  std::cout << "MS-GNC-TLS:" << "\n" << ms_gnc_tls_reg << "\n\n";

  registration::Params fracgm_params;
  fracgm_params.max_iteration = 100;
  fracgm_params.tolerance = 1e-6;
  fracgm_params.threshold_c = 1.0;
  auto fracgm_reg = registration::FracgmSolver(fracgm_params).solve(inlier_src_reg, inlier_dst_reg, noise_bound);
  std::cout << "FracGM:" << "\n" << fracgm_reg << "\n\n";

  return 0;
}