# Geman-McClure Robust Registration

[![C++ Formatter](https://img.shields.io/badge/C++_Formatter-clang--format_18.1.3-blue?style=flat-square)](https://github.com/llvm/llvm-project/releases/tag/llvmorg-18.1.3)
[![Python Formatter](https://img.shields.io/badge/Python_Formatter-ruff-red?style=flat-square)](https://github.com/astral-sh/ruff)


**About**

Official implementation of my master thesis "Algorithms for Geman-McClure Robust Estimation and Applications for Spatial Perceptions". This library is written in **C++** and we support **C++**, and **Python** interfaces.

**Table of Contents**

- [Geman-McClure Robust Registration](#geman-mcclure-robust-registration)
  - [:gear: Setup](#gear-setup)
    - [Prerequisites](#prerequisites)
    - [Build](#build)
    - [(optional) Build with Python binding](#optional-build-with-python-binding)
  - [:books: Example Usages](#books-example-usages)
  - [:card\_file\_box: Related works](#card_file_box-related-works)
  - [:gift: Acknowledgement](#gift-acknowledgement)

## :gear: Setup

The following setup is tested in Ubuntu 22.04.

### Prerequisites
```shell
sudo apt update
sudo apt install -y g++ build-essential cmake
sudo apt install -y libeigen3-dev libomp-dev
# python (optional)
sudo apt install -y python3 python3-dev python3-venv

git clone --recurse-submodules -j8 https://github.com/doggydoggy0101/master-thesis.git
cd master-thesis

# If you do not clone the repository with --recurse-submodules option,
# you may need to run the following command.
git submodule update --init
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
python3 -m venv .venv
source .venv/bin/activate
pip install numpy

mkdir build
cd build
cmake .. -DBUILD_PYTHON=ON -DPYTHON_EXECUTABLE=$(which python3)
make

cd python && pip install .
```

## :books: Example Usages

We provide two robust point cloud registration solvers.

| Solver | Description                                                         |
| ------ | ------------------------------------------------------------------- |
| FracGM | A FracGM-based registration solver with linear relaxation approach. |
| QGM    | A QGM-based registration solver with linear relaxation approach.    |

- [:croissant: C++](examples/cpp)
- [:snake: Python](examples/python)

Implementation of other solvers used in the synthetic dataset experiments can be found [here](https://github.com/doggydoggy0101/registration).


## :card_file_box: Related works

- [Bang-Shien Chen](https://dgbshien.com/), [Yu-Kai Lin](https://github.com/StephLin), [Jian-Yu Chen](https://github.com/Jian-yu-chen), [Chih-Wei Huang](https://sites.google.com/ce.ncu.edu.tw/cwhuang/), [Jann-Long Chern](https://math.ntnu.edu.tw/~chern/), [Ching-Cherng Sun](https://www.dop.ncu.edu.tw/en/Faculty/faculty_more/9), **FracGM: A Fast Fractional Programming Technique for Geman-McClure Robust Estimator**. _IEEE Robotics and Automation Letters (RA-L)_, vol. 9, no. 12, pp. 11666-11673, Dec. 2024. ([paper](https://doi.org/10.1109/lra.2024.3495372)) ([preprint](https://arxiv.org/pdf/2409.13978)) ([code](https://github.com/StephLin/FracGM))


## :gift: Acknowledgement

- The feature of maximum clique inlier selection (MCIS) refers to [TEASER++](https://github.com/MIT-SPARK/TEASER-plusplus), which is under the MIT license.
- The dataset is from [3DMatch](https://3dmatch.cs.princeton.edu/).
