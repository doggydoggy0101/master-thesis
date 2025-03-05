/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <string>

namespace registration {

struct Params {
  size_t max_iteration = 100;  // The maximum number of iterations allowed.
  double tolerance = 1e-6;     // The tolerance for convergence.
  std::string robust_type =
      "GM";                  // Robust function: Truncated Least Squares (TLS), Geman-McClure (GM) or L0-norm (L0).
  double threshold_c = 1.0;  // The parameter $c$ of the robust function.
  double gnc_factor = 1.4;   // Surrogate parameter's update step size.
  double weight_tolerance = 1e-4;  // Stopping critera for weights being binary.
  bool majorization = true;        // Use majorization surrogate for TLS (does nothing for other robust function).
  bool superlinear =
      true;  // Use superlienar update surrogate parameter for TLS (does nothing for other robust function).
};

}  // namespace registration