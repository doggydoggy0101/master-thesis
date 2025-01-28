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


def perform_mcis(src, dst, noise_bound, pmc_timeout, pmc_n_threads):
    indices = registration_python.outlier_rejection.maximum_clique_inlier_selection(
        src, dst, noise_bound, pmc_timeout, pmc_n_threads
    )
    return np.take(src, indices, axis=0), np.take(dst, indices, axis=0)


def perform_tuple(src, dst, tuple_scale, max_tuple_count):
    indices = registration_python.outlier_rejection.tuple_test(
        src, dst, tuple_scale, max_tuple_count
    )
    return np.take(src, indices, axis=0), np.take(dst, indices, axis=0)


def main():
    max_iteration = 100
    tol = 1e-6
    c = 1.0
    noise_bound = 0.1
    gnc_factor = 1.4
    weight_tol = 1e-4

    # outlier rejection
    method = "mcis"  # "mcis" or "tuple"
    # mcis
    pmc_timeout = 3600.0
    pmc_n_threads = 4
    # tuple
    tuple_scale = 0.95
    max_tuple_count = 1000

    src_reg, dst_reg, gt_reg = get_toy_data()

    if method == "mcis":
        src_reg, dst_reg = perform_mcis(
            src_reg, dst_reg, noise_bound, pmc_timeout, pmc_n_threads
        )
    elif method == "tuple":
        src_reg, dst_reg = perform_tuple(src_reg, dst_reg, tuple_scale, max_tuple_count)

    irls_tls_reg = registration_python.IrlsSolver(max_iteration, tol, "TLS", c).solve(
        src_reg, dst_reg, noise_bound
    )
    irls_gm_reg = registration_python.IrlsSolver(max_iteration, tol, "GM", c).solve(
        src_reg, dst_reg, noise_bound
    )
    gnc_tls_reg = registration_python.GncSolver(max_iteration, tol, "TLS", c, gnc_factor, weight_tol).solve(
        src_reg, dst_reg, noise_bound
    )
    gnc_gm_reg = registration_python.GncSolver(max_iteration, tol, "GM", c, gnc_factor, weight_tol).solve(
        src_reg, dst_reg, noise_bound
    )
    fracgm_reg = registration_python.FracgmSolver(max_iteration, tol, c).solve(
        src_reg, dst_reg, noise_bound
    )

    print("Ground truth:")
    print(gt_reg, end="\n\n")

    print("IRLS-TLS:")
    print(irls_tls_reg, end="\n\n")

    print("IRLS-GM:")
    print(irls_gm_reg, end="\n\n")

    print("GNC-TLS:")
    print(gnc_tls_reg, end="\n\n")

    print("GNC-GM:")
    print(gnc_gm_reg, end="\n\n")

    print("FracGM:")
    print(fracgm_reg, end="\n\n")


if __name__ == "__main__":
    main()
