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


def perform_mcis(pcd1, pcd2, noise_bound, pmc_timeout, pmc_n_threads):
    indices = registration_python.outlier_rejection.maximum_clique_inlier_selection(
        pcd1,
        pcd2,
        noise_bound=noise_bound,
        pmc_timeout=pmc_timeout,
        pmc_n_threads=pmc_n_threads,
    )
    return np.take(pcd1, indices, axis=0), np.take(pcd2, indices, axis=0)


def perform_tuple(pcd1, pcd2, tuple_scale, max_tuple_count):
    indices = registration_python.outlier_rejection.tuple_test(
        pcd1, pcd2, tuple_scale=tuple_scale, max_tuple_count=max_tuple_count
    )
    return np.take(pcd1, indices, axis=0), np.take(pcd2, indices, axis=0)


def main():
    max_iteration = 100
    tol = 1e-6
    c = 1.0
    noise_bound = 0.1

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
            pcd1=src_reg,
            pcd2=dst_reg,
            noise_bound=noise_bound,
            pmc_timeout=pmc_timeout,
            pmc_n_threads=pmc_n_threads,
        )
    elif method == "tuple":
        src_reg, dst_reg = perform_tuple(
            pcd1=src_reg,
            pcd2=dst_reg,
            tuple_scale=tuple_scale,
            max_tuple_count=max_tuple_count,
        )

    irls_tls_reg = registration_python.IrlsSolver(
        max_iteration=max_iteration, tolerance=tol, robust_type="TLS", threshold_c=c
    ).solve(pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound)
    irls_gm_reg = registration_python.IrlsSolver(
        max_iteration=max_iteration, tolerance=tol, robust_type="GM", threshold_c=c
    ).solve(pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound)
    gnc_tls_reg = registration_python.GncSolver(
        max_iteration=max_iteration, tolerance=tol, robust_type="TLS", threshold_c=c
    ).solve(pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound)
    gnc_gm_reg = registration_python.GncSolver(
        max_iteration=max_iteration, tolerance=tol, robust_type="GM", threshold_c=c
    ).solve(pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound)
    fracgm_reg = registration_python.FracgmSolver(
        max_iteration=max_iteration, tolerance=tol, threshold_c=c
    ).solve(pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound)

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
