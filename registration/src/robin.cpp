/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include "registration/robin.h"

std::vector<size_t> registration::outlier_rejection::robin(const PointCloud& src_point_cloud,
                                                           const PointCloud& tar_point_cloud, double noise_bound,
                                                           std::string robin_mode) {
  // ROBIN's input pcd shape is (3, n)
  auto* g = robin::Make3dRegInvGraph(src_point_cloud.transpose(), tar_point_cloud.transpose(), noise_bound);

  // borrowed from KISS-Matcher
  auto corres = [&]() {
    // NOTE(hlim): Just use max core mode.
    // `max_clique` not only took more time but also showed slightly worse performance.
    if (robin_mode == "max_core") {
      return robin::FindInlierStructure(g, robin::InlierGraphStructure::MAX_CORE);
    } else if (robin_mode == "max_clique") {
      return robin::FindInlierStructure(g, robin::InlierGraphStructure::MAX_CLIQUE);
    } else {
      throw std::runtime_error("Something's wrong!");
    }
  }();
  std::sort(corres.begin(), corres.end());

  delete g;

  return corres;
}