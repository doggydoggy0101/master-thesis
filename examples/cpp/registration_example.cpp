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

// #include "registration/mcis.h"
#include "registration/fracgm.h"
#include "registration/qgm.h"

// #define ENABLE_MAX_CLIQUE_INLIER_SELECTION

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

// TODO: MCIS

int main() {
  size_t max_iteration = 100;
  double tol = 1e-6;
  double c = 1.0;
  double noise_bound = 0.1;

  double pmc_timeout = 3600.0;
  int pmc_n_threads = 4;

  auto [src_reg, dst_reg, gt_reg] = get_toy_data();

  auto fracgm_reg = registration::fracgm(max_iteration, tol, c, noise_bound).solve(src_reg, dst_reg);
  auto qgm_reg = registration::qgm(max_iteration, tol, c, noise_bound).solve(src_reg, dst_reg);
  
  std::cout << "Ground Truth:" << "\n" << gt_reg << "\n\n";
  std::cout << "FracGM:" << "\n" << fracgm_reg << "\n\n";
  std::cout << "QGM:" << "\n" << qgm_reg << "\n\n";

  return 0;
}