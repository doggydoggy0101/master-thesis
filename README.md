# Robust Registration

[![C++ Formatter](https://img.shields.io/badge/C++_Formatter-clang--format_18.1.3-blue?style=flat-square)](https://github.com/llvm/llvm-project/releases/tag/llvmorg-18.1.3)
[![Python Formatter](https://img.shields.io/badge/Python_Formatter-ruff-red?style=flat-square)](https://github.com/astral-sh/ruff)

Official implementation of my master thesis "Algorithms for Geman-McClure Robust Estimation and Applications for Spatial Perceptions". It has since been extended with state-of-the-art robust registration solvers. This library is written in **C++** and we support **Python** interface.

| ![](docs/bunny_iterations.gif)   | ![](docs/bunny_results.gif)   |
| -------------------------------- | ----------------------------- |
| ![](docs/3dmatch_iterations.gif) | ![](docs/3dmatch_results.gif) |

> Point cloud registration example of our FracgmSolver.

**Table of Contents**

- [Robust Registration](#robust-registration)
  - [:gear: Setup](#gear-setup)
    - [Prerequisites](#prerequisites)
    - [Build](#build)
    - [(optional) Build with Python binding](#optional-build-with-python-binding)
  - [:books: Example usages](#books-example-usages)
  - [:video\_game: Numerical results](#video_game-numerical-results)
  - [:construction: TODO](#construction-todo)
  - [:card\_file\_box: Related works](#card_file_box-related-works)

## :gear: Setup

The following setup is tested in Ubuntu 22.04.

### Prerequisites
```shell
sudo apt update
sudo apt install -y g++ build-essential cmake
sudo apt install -y libeigen3-dev libomp-dev
```


### Build

```shell
mkdir build
cd build
cmake .. 
make
```

### (optional) Build with Python binding
```shell
sudo apt install -y python3 python3-dev python3-venv
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install --upgrade pip
python3 -m pip install numpy

mkdir build
cd build
cmake .. -DBUILD_PYTHON=ON
make

cd python && pip install .
```

## :books: Example usages

- [:croissant: C++](examples/cpp)
- [:snake: Python](examples/python)

We provide the following outlier rejection methods:

| Method | Description                                                                                   |
|--------|-----------------------------------------------------------------------------------------------|
| tuple  | Tuple test from [FGR](https://github.com/isl-org/FastGlobalRegistration).                     |
| mcis   | Maximum clique inlier selction from [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus). |
| robin  | Graph theoretic outlier rejction from [ROBIN](https://github.com/MIT-SPARK/ROBIN).            |

We provide the following robust point cloud registration solvers:

| Solver       | Robust function | Description                                              |
|--------------|-----------------|----------------------------------------------------------|
| IrlsSolver   | TLS, GM         | IRLS-based registration solver with linear relaxation.   |
| GncSolver    | TLS, GM, L0     | GNC-based registration solver with linear relaxation.    |
| FracgmSolver | GM              | FracGM-based registration solver with linear relaxation. |

We point out that all robust solvers are iterative solvers and requires some inner solver for the registration problem. 
[TEASER](https://github.com/MIT-SPARK/TEASER-plusplus) decouples the registration problem to a rotation and translation subproblem, where the rotation subproblem is solved by Singular Value Decomposition and the translation subproblem has a closed-form. However, this closed-form is for TLS only. In contrast, [FracGM](https://github.com/StephLin/FracGM) applies linear relaxation, where the registration problem has a closed-form on the Euclidean space, then project the solution back to the feasible set at last. I personally believe linear relaxation is a better approach in point cloud registration because 

1. invariant measurements for the rotation subproblem increases exponentially,
2. the translation subproblem relies on the quality of the estimated rotation,
3. it can be easily adapted to any iterative solver. 
   
A comparison between decouple (TEASER++) and linear relaxation (GNC-TLS) can be found in the table below.

> Note that QGM in my master thesis is equivalent to IrlsSolver with the Geman-McClure robust function. Implementation of other solvers used in the synthetic dataset experiments can be found [here](https://github.com/doggydoggy0101/registration). 


## :video_game: Numerical results

We benchmark the solvers against [RANSAC](https://github.com/isl-org/Open3D), [FGR](https://github.com/isl-org/FastGlobalRegistration), and [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus) on the [3DMatch](https://3dmatch.cs.princeton.edu/) dataset. We generate correspondeces with FPFH and choose `mcis` as the outlier rejection method for all solvers in this experiment.

![images](docs/3dmatch.png)

Minimal set-up for this experiment (needs Anaconda installed):

```sh
conda create -n reg python=3.11 "numpy<2.0" -y
conda activate reg
# FGR (and FPFH for correspondences)
pip install open3d
# TEASER (currently does not support numpy>=2.0)
git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DTEASERPP_PYTHON_VERSION=3.11 && make teaserpp_python -j24
cd python && pip install . && cd ../../..
# robust registration solvers
git clone https://github.com/doggydoggy0101/master-thesis.git
cd master-thesis && mkdir build && cd build
cmake .. -DBUILD_PYTHON=ON && make -j24
cd python && pip install . && cd ../../..
```


## :construction: TODO

- Add support for more robust functions to IrlsSolver.
- Add generate correspondeces methods such as FPFH and FasterPFH.

I aim to make this project a go-to choice for point cloud registration.
Feel free to open an issue or PR if you arere interested in contributing!



## :card_file_box: Related works

- Bang-Shien Chen, Yu-Kai Lin, Jian-Yu Chen, Chih-Wei Huang, Jann-Long Chern, Ching-Cherng Sun. **FracGM: A Fast Fractional Programming Technique for Geman-McClure Robust Estimator**. _IEEE Robotics and Automation Letters (RA-L)_, vol. 9, no. 12, pp. 11666-11673, 2024. ([paper](https://doi.org/10.1109/lra.2024.3495372)) ([preprint](https://arxiv.org/pdf/2409.13978)) ([code](https://github.com/StephLin/FracGM))

- Liangzu Peng, Christian Kümmerle, René Vidal, **On the convergence of IRLS and its variants in outlier-robust estimation**. _In Proceedings of the IEEE/CVF Conference on Computer Vision and Pattern Recognition_, pp. 17808-17818, 2023. ([paper](https://doi.org/10.1109/CVPR52729.2023.01708)) ([preprint](https://openaccess.thecvf.com/content/CVPR2023/papers/Peng_On_the_Convergence_of_IRLS_and_Its_Variants_in_Outlier-Robust_CVPR_2023_paper.pdf)) ([code](https://github.com/liangzu/IRLS-CVPR2023))

- Heng Yang, Pasquale Antonante, Vasileios Tzoumas, Luca Carlone. **Graduated Non-Convexity for Robust Spatial Perception: From Non-Minimal Solvers to Global Outlier Rejection**. _IEEE Robotics and Automation Letters_, vol. 5, no. 2, pp. 1127-1134, 2020. ([paper](https://doi.org/10.1109/LRA.2020.2965893))([preprint](https://arxiv.org/pdf/1909.08605))([code](https://github.com/MIT-SPARK/GNC-and-ADAPT))