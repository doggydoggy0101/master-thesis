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

#include "registration/mcis.h"

#include "pmc/pmc.h"

std::vector<int> registration::mcis::MaxCliqueSolver::findMaxClique(registration::mcis::Graph graph) {
  // Handle deprecated field
  if (!params_.solve_exactly) {
    params_.solver_mode = CLIQUE_SOLVER_MODE::PMC_HEU;
  }

  // Create a PMC graph from the TEASER graph
  std::vector<int> edges;
  std::vector<long long> vertices;
  vertices.push_back(edges.size());

  const auto all_vertices = graph.getVertices();
  for (const auto& i : all_vertices) {
    const auto& c_edges = graph.getEdges(i);
    edges.insert(edges.end(), c_edges.begin(), c_edges.end());
    vertices.push_back(edges.size());
  }

  // Use PMC to calculate
  pmc::pmc_graph G(vertices, edges);

  // Prepare PMC input
  // TODO: Incorporate this to the constructor
  pmc::input in;
  in.algorithm = 0;
  in.threads = params_.num_threads;
  in.experiment = 0;
  in.lb = 0;
  in.ub = 0;
  in.param_ub = 0;
  in.adj_limit = 20000;
  in.time_limit = params_.time_limit;
  in.remove_time = 4;
  in.graph_stats = false;
  in.verbose = false;
  in.help = false;
  in.MCE = false;
  in.decreasing_order = false;
  in.heu_strat = "kcore";
  in.vertex_search_order = "deg";

  // vector to represent max clique
  std::vector<int> C;

  // upper-bound of max clique
  G.compute_cores();
  auto max_core = G.get_max_core();

  // TEASER_DEBUG_INFO_MSG("Max core number: " << max_core);
  // TEASER_DEBUG_INFO_MSG("Num vertices: " << vertices.size());

  // check for k-core heuristic threshold
  // check whether threshold equals 1 to short circuit the comparison
  if (params_.solver_mode == CLIQUE_SOLVER_MODE::KCORE_HEU && params_.kcore_heuristic_threshold != 1 &&
      max_core > static_cast<int>(params_.kcore_heuristic_threshold * static_cast<double>(all_vertices.size()))) {
    // TEASER_DEBUG_INFO_MSG("Using K-core heuristic finder.");
    // remove all nodes with core number less than max core number
    // k_cores is a vector saving the core number of each vertex
    auto k_cores = G.get_kcores();
    for (int i = 1; i < k_cores->size(); ++i) {
      // Note: k_core has size equals to num vertices + 1
      if ((*k_cores)[i] >= max_core) {
        C.push_back(i - 1);
      }
    }
    return C;
  }

  if (in.ub == 0) {
    in.ub = max_core + 1;
  }

  // lower-bound of max clique
  if (in.lb == 0 && in.heu_strat != "0") {  // skip if given as input
    pmc::pmc_heu maxclique(G, in);
    in.lb = maxclique.search(G, C);
  }

  assert(in.lb != 0);
  if (in.lb == 0) {
    // This means that max clique has a size of one
    // TEASER_DEBUG_ERROR_MSG("Max clique lower bound equals to zero. Abort.");
    return C;
  }

  if (in.lb == in.ub) {
    return C;
  }

  // Optional exact max clique finding
  if (params_.solver_mode == CLIQUE_SOLVER_MODE::PMC_EXACT) {
    // The following methods are used:
    // 1. k-core pruning
    // 2. neigh-core pruning/ordering
    // 3. dynamic coloring bounds/sort
    // see the original PMC paper and implementation for details:
    // R. A. Rossi, D. F. Gleich, and A. H. Gebremedhin, “Parallel Maximum Clique Algorithms with
    // Applications to Network Analysis,” SIAM J. Sci. Comput., vol. 37, no. 5, pp. C589–C616, Jan.
    // 2015.
    if (G.num_vertices() < in.adj_limit) {
      G.create_adj();
      pmc::pmcx_maxclique finder(G, in);
      finder.search_dense(G, C);
    } else {
      pmc::pmcx_maxclique finder(G, in);
      finder.search(G, C);
    }
  }

  return C;
}

Eigen::Matrix<double, 3, Eigen::Dynamic> compute_tims(const registration::PointCloud& v,
                                                      Eigen::Matrix<int, 2, Eigen::Dynamic>* map) {
  auto N = v.rows();
  Eigen::Matrix<double, 3, Eigen::Dynamic> vtilde(3, N * (N - 1) / 2);
  map->resize(2, N * (N - 1) / 2);

#pragma omp parallel for default(none) shared(N, v, vtilde, map)
  for (size_t i = 0; i < N - 1; i++) {
    // Calculate some important indices
    // For each measurement, we compute the TIMs between itself and all the measurements after it.
    // For example:
    // i=0: add N-1 TIMs
    // i=1: add N-2 TIMs
    // etc..
    // i=k: add N-1-k TIMs
    // And by arithmatic series, we can get the starting index of each segment be:
    // k*N - k*(k+1)/2
    size_t segment_start_idx = i * N - i * (i + 1) / 2;
    size_t segment_cols = N - 1 - i;

    // calculate TIM
    Eigen::Matrix<double, 3, 1> m = v.row(i).transpose();
    Eigen::Matrix<double, 3, Eigen::Dynamic> temp = v.transpose() - m * Eigen::MatrixXd::Ones(1, N);

    // concatenate to the end of the tilde vector
    vtilde.middleCols(segment_start_idx, segment_cols) = temp.rightCols(segment_cols);

    // populate the index map
    Eigen::Matrix<int, 2, Eigen::Dynamic> map_addition(2, N);
    for (size_t j = 0; j < N; ++j) {
      map_addition(0, j) = i;
      map_addition(1, j) = j;
    }
    map->middleCols(segment_start_idx, segment_cols) = map_addition.rightCols(segment_cols);
  }

  return vtilde;
}

void scale_inliers_selector(const Eigen::Matrix<double, 3, Eigen::Dynamic>& src,
                            const Eigen::Matrix<double, 3, Eigen::Dynamic>& dst, double noise_bound,
                            Eigen::Matrix<bool, 1, Eigen::Dynamic>* inliers) {
  Eigen::Matrix<double, 1, Eigen::Dynamic> v1_dist = src.array().square().colwise().sum().array().sqrt();
  Eigen::Matrix<double, 1, Eigen::Dynamic> v2_dist = dst.array().square().colwise().sum().array().sqrt();
  double beta = 2 * noise_bound;  // * sqrt(cbar2_)  // Skiped, which is typically 1.0

  // A pair-wise correspondence is an inlier if it passes the following test:
  // abs(|dst| - |src|) is within maximum allowed error
  *inliers = (v1_dist.array() - v2_dist.array()).array().abs() <= beta;
}

std::vector<int> registration::outlier_rejection::mcis(const registration::PointCloud& src,
                                                       const registration::PointCloud& dst, double noise_bound,
                                                       double pmc_timeout, int pmc_n_threads) {
  Eigen::Matrix<int, 2, Eigen::Dynamic> src_tims_map_;
  Eigen::Matrix<int, 2, Eigen::Dynamic> dst_tims_map_;

  auto src_tims_ = compute_tims(src, &src_tims_map_);
  auto dst_tims_ = compute_tims(dst, &dst_tims_map_);

  Eigen::Matrix<bool, 1, Eigen::Dynamic> scale_inliers_mask_;

  // TEASER_DEBUG_INFO_MSG("Starting scale solver (only selecting inliers if scale estimation has been disabled).");
  scale_inliers_selector(src_tims_, dst_tims_, noise_bound, &scale_inliers_mask_);
  // TEASER_DEBUG_INFO_MSG("Scale estimation complete.");

  // Calculate Maximum Clique (PMC_EXACT)
  // Note: the max_clique_ vector holds the indices of original measurements that are within the
  // max clique of the built inlier graph.

  // Create inlier graph: A graph with (indices of) original measurements as vertices, and edges
  // only when the TIM between two measurements are inliers. Note: src_tims_map_ is the same as
  // dst_tim_map_
  registration::mcis::Graph inlier_graph_;

  inlier_graph_.populateVertices(src.rows());
  for (size_t i = 0; i < scale_inliers_mask_.cols(); ++i) {
    if (scale_inliers_mask_(0, i)) {
      inlier_graph_.addEdge(src_tims_map_(0, i), src_tims_map_(1, i));
    }
  }

  registration::mcis::MaxCliqueSolver::Params clique_params;

  clique_params.solver_mode = registration::mcis::MaxCliqueSolver::CLIQUE_SOLVER_MODE::PMC_EXACT;
  clique_params.time_limit = pmc_timeout;
  clique_params.num_threads = pmc_n_threads;

  registration::mcis::MaxCliqueSolver clique_solver(clique_params);
  auto max_clique_ = clique_solver.findMaxClique(inlier_graph_);
  std::sort(max_clique_.begin(), max_clique_.end());
  // TEASER_DEBUG_INFO_MSG("Max Clique of scale estimation inliers: ");

  return max_clique_;
}