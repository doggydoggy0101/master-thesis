# Copyright 2025 Bang-Shien Chen.
# All rights reserved. See LICENSE for the license information.

import os
import sys
import numpy as np

__dir__ = os.path.dirname(os.path.abspath(__file__))
try:
    path = os.path.join(__dir__, "../../build/python")
    sys.path.append(path)
    print(path)
    import registration_python
except ImportError:
    print("Make sure project was built with python binding.")
    exit(1)


CLOUD_SRC_PATH = "../data/cloud_bin_0.txt"
CLOUD_DST_PATH = "../data/cloud_bin_1.txt"
GT_PATH = "../data/gt.txt"


def get_toy_data():
    src = np.loadtxt(CLOUD_SRC_PATH)
    dst = np.loadtxt(CLOUD_DST_PATH)

    gt = np.loadtxt(GT_PATH)

    return src, dst, gt


def main():
    max_iteration = 100
    tol = 1e-6
    c = 1.0
    noise_bound = 0.1

    ENABLE_MAX_CLIQUE_INLIER_SELECTION = True
    pmc_timeout = 3600.0
    pmc_n_threads = 4
    src_reg, dst_reg, gt_reg = get_toy_data()

    # TODO: MCIS

    fracgm_reg = registration_python.fracgm(max_iteration, tol, c, noise_bound).solve(
        src_reg, dst_reg
    )
    qgm_reg = registration_python.qgm(max_iteration, tol, c, noise_bound).solve(
        src_reg, dst_reg
    )

    print("Ground truth:")
    print(gt_reg, end="\n\n")

    print("FracGM:")
    print(fracgm_reg, end="\n\n")

    print("QGM:")
    print(qgm_reg, end="\n\n")


if __name__ == "__main__":
    main()
