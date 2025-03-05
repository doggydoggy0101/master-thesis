# Copyright 2025 Bang-Shien Chen.
# All rights reserved. See LICENSE for the license information.

import numpy as np
import registration_python

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
    # data assumption
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
    print("Ground truth:")
    print(gt_reg, end="\n\n")

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

    irls_tls_params = registration_python.Params()
    irls_tls_params.max_iteration = 100
    irls_tls_params.tolerance = 1e-6
    irls_tls_params.robust_type = "TLS"
    irls_tls_params.threshold_c = 1.0
    irls_tls_reg = registration_python.IrlsSolver(irls_tls_params).solve(
        pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound
    )
    print("IRLS-TLS:")
    print(irls_tls_reg, end="\n\n")

    irls_gm_params = registration_python.Params()
    irls_gm_params.max_iteration = 100
    irls_gm_params.tolerance = 1e-6
    irls_gm_params.robust_type = "GM"
    irls_gm_params.threshold_c = 1.0
    irls_gm_reg = registration_python.IrlsSolver(irls_gm_params).solve(
        pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound
    )
    print("IRLS-GM:")
    print(irls_gm_reg, end="\n\n")

    gnc_tls_params = registration_python.Params()
    gnc_tls_params.max_iteration = 100
    gnc_tls_params.tolerance = 1e-6
    gnc_tls_params.robust_type = "TLS"
    gnc_tls_params.threshold_c = 1.0
    gnc_tls_reg = registration_python.GncSolver(gnc_tls_params).solve(
        pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound
    )
    print("GNC-TLS:")
    print(gnc_tls_reg, end="\n\n")

    gnc_gm_params = registration_python.Params()
    gnc_gm_params.max_iteration = 100
    gnc_gm_params.tolerance = 1e-6
    gnc_gm_params.robust_type = "GM"
    gnc_gm_params.threshold_c = 1.0
    gnc_gm_reg = registration_python.GncSolver(gnc_gm_params).solve(
        pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound
    )
    print("GNC-GM:")
    print(gnc_gm_reg, end="\n\n")

    ms_gnc_l0_params = registration_python.Params()
    ms_gnc_l0_params.max_iteration = 100
    ms_gnc_l0_params.tolerance = 1e-6
    ms_gnc_l0_params.robust_type = "L0"
    ms_gnc_l0_params.threshold_c = 1.0
    ms_gnc_l0_reg = registration_python.GncSolver(ms_gnc_l0_params).solve(
        pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound
    )
    print("MS-GNC-L0:")
    print(ms_gnc_l0_reg, end="\n\n")

    ms_gnc_tls_params = registration_python.Params()
    ms_gnc_tls_params.max_iteration = 100
    ms_gnc_tls_params.tolerance = 1e-6
    ms_gnc_tls_params.robust_type = "TLS"
    ms_gnc_tls_params.threshold_c = 1.0
    ms_gnc_tls_params.majorization = True
    ms_gnc_tls_params.superlinear = True
    ms_gnc_tls_reg = registration_python.GncSolver(ms_gnc_tls_params).solve(
        pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound
    )
    print("MS-GNC-TLS:")
    print(ms_gnc_tls_reg, end="\n\n")

    fracgm_params = registration_python.Params()
    fracgm_params.max_iteration = 100
    fracgm_params.tolerance = 1e-6
    fracgm_params.threshold_c = 1.0
    fracgm_reg = registration_python.FracgmSolver(fracgm_params).solve(
        pcd1=src_reg, pcd2=dst_reg, noise_bound=noise_bound
    )
    print("FracGM:")
    print(fracgm_reg, end="\n\n")


if __name__ == "__main__":
    main()
