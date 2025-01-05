# Copyright 2025 Bang-Shien Chen.
# All rights reserved. See LICENSE for the license information.

import numpy as np

try:
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


def perform_max_clique_inlier_selection(
    src, dst, noise_bound, pmc_timeout, pmc_n_threads
):
    indices = registration_python.mcis.inlier_selection(
        src, dst, noise_bound, pmc_timeout, pmc_n_threads
    )
    return np.take(src, indices, axis=0), np.take(dst, indices, axis=0)


def main():
    max_iteration = 100
    tol = 1e-6
    c = 1.0
    noise_bound = 0.1

    ENABLE_MAX_CLIQUE_INLIER_SELECTION = True
    pmc_timeout = 3600.0
    pmc_n_threads = 4
    src_reg, dst_reg, gt_reg = get_toy_data()

    if ENABLE_MAX_CLIQUE_INLIER_SELECTION:
        src_reg, dst_reg = perform_max_clique_inlier_selection(
            src_reg, dst_reg, noise_bound, pmc_timeout, pmc_n_threads
        )

    fracgm_reg = registration_python.FracGM(max_iteration, tol, c).solve(
        src_reg, dst_reg, noise_bound
    )
    qgm_reg = registration_python.QGM(max_iteration, tol, c).solve(
        src_reg, dst_reg, noise_bound
    )

    print("Ground truth:")
    print(gt_reg, end="\n\n")

    print("FracGM:")
    print(fracgm_reg, end="\n\n")

    print("QGM:")
    print(qgm_reg, end="\n\n")


if __name__ == "__main__":
    main()
