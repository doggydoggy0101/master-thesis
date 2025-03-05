/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <string>

namespace registration {
/**
 * @param max_iteration The maximum number of iterations allowed.
 * @param tolerance The tolerance for convergence.
 * @param robust_type Robust function: Truncated Least Squares (TLS), Geman-McClure (GM) or L0-norm (L0).
 * @param threshold_c The parameter $c$ of the robust function.
 * @param gnc_factor Surrogate parameter's update step size.
 * @param weight_tolerance Stopping critera for weights being binary.
 * @param majorization Use majorization surrogate for TLS (does nothing for other robust function).
 * @param superlinear Use superlienar update surrogate parameter for TLS (does nothing for other robust function).
 */
struct Params {
  size_t max_iteration = 100; 
  double tolerance = 1e-6; 
  std::string robust_type = "GM";
  double threshold_c = 1.0; 
  double gnc_factor = 1.4; 
  double weight_tolerance = 1e-4;
  bool majorization = false; 
  bool superlinear = false;
};

}  // namespace registration