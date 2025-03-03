# Geman-McClure Robust Registration

[![C++ Formatter](https://img.shields.io/badge/C++_Formatter-clang--format_18.1.3-blue?style=flat-square)](https://github.com/llvm/llvm-project/releases/tag/llvmorg-18.1.3)
[![Python Formatter](https://img.shields.io/badge/Python_Formatter-ruff-red?style=flat-square)](https://github.com/astral-sh/ruff)


**About**

Official implementation of my master thesis "Algorithms for Geman-McClure Robust Estimation and Applications for Spatial Perceptions". This library is written in **C++** and we support **Python** interface.

**Table of Contents**

- [Geman-McClure Robust Registration](#geman-mcclure-robust-registration)
  - [:gear: Setup](#gear-setup)
    - [Prerequisites](#prerequisites)
    - [Build](#build)
    - [(optional) Build with Python binding](#optional-build-with-python-binding)
  - [:books: Example usages](#books-example-usages)
  - [:video\_game: Numerical results](#video_game-numerical-results)
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

We provide the following robust point cloud registration solvers:

| Solver       | Robust   | Description                                                       |
|--------------|----------|-------------------------------------------------------------------|
| IrlsSolver   | TLS & GM | IRLS-based registration solver with linear relaxation approach.   |
| GncSolver    | TLS & GM | GNC-based registration solver with linear relaxation approach.    |
| FracgmSolver | GM       | FracGM-based registration solver with linear relaxation approach. |

> Note that QGM is IrlsSolver with the Geman-McClure robust function.

- [:croissant: C++](examples/cpp)
- [:snake: Python](examples/python)

Implementation of other solvers used in the synthetic dataset experiments can be found [here](https://github.com/doggydoggy0101/registration). 


## :video_game: Numerical results

We benchmark the solvers against [RANSAC](https://github.com/isl-org/Open3D), [FGR](https://github.com/isl-org/FastGlobalRegistration), and [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus) on the [3DMatch](https://3dmatch.cs.princeton.edu/) dataset. All experiments are conducted over 40 Monte Carlo runs.

![images](docs/3dmatch.png)

> TEASER++ has two runtime patterns, some runs complete with (~0.0005s), while others take (~0.2-0.4s) for the same scene. Despite these runtime differences, the final rotation and translation errors remain consistent across runs, which suggests that TEASER++ is completing the registration successfully in all cases. I am still looking for the reason why this happens.


## :card_file_box: Related works

- [Bang-Shien Chen](https://dgbshien.com/), [Yu-Kai Lin](https://github.com/StephLin), [Jian-Yu Chen](https://github.com/Jian-yu-chen), [Chih-Wei Huang](https://sites.google.com/ce.ncu.edu.tw/cwhuang/), [Jann-Long Chern](https://math.ntnu.edu.tw/~chern/), [Ching-Cherng Sun](https://www.dop.ncu.edu.tw/en/Faculty/faculty_more/9), **FracGM: A Fast Fractional Programming Technique for Geman-McClure Robust Estimator**. _IEEE Robotics and Automation Letters (RA-L)_, vol. 9, no. 12, pp. 11666-11673, Dec. 2024. ([paper](https://doi.org/10.1109/lra.2024.3495372)) ([preprint](https://arxiv.org/pdf/2409.13978)) ([code](https://github.com/StephLin/FracGM))
