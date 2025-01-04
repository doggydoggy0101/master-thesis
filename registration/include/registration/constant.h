/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#pragma once

#include <Eigen/Dense>

namespace registration {

const Eigen::Vector<double, 13> e = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};

const Eigen::MatrixXd eye3 = Eigen::MatrixXd::Identity(3, 3);

const Eigen::MatrixXd eye13 = Eigen::MatrixXd::Identity(13, 13);

};  // namespace registration