# Geman-McClure Robust Registration

[![Python Formatter](https://img.shields.io/badge/Python_Formatter-ruff-black?style=flat-square)](https://github.com/astral-sh/ruff)
[![C++ Formatter](https://img.shields.io/badge/C++_Formatter-clang--format_18.1.8-lightblue?style=flat-square)](https://releases.llvm.org/18.1.8/tools/clang/tools/extra/docs/ReleaseNotes.html)


**About**

TBD

**Table of Contents**

- [Geman-McClure Robust Registration](#geman-mcclure-robust-registration)
  - [:gear: Setup](#gear-setup)
    - [Prerequisites](#prerequisites)
    - [Build](#build)
    - [(optional) Build with Python binding](#optional-build-with-python-binding)
  - [:books: Example Usages](#books-example-usages)

## :gear: Setup

The following setup is tested in Ubuntu 22.04.

### Prerequisites
```shell
sudo apt update
sudo apt install -y g++ build-essential cmake
sudo apt install -y libeigen3-dev libomp-dev
# python (optional)
sudo apt install -y python3 python3-dev
python3 -m pip install numpy 

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
mkdir build
cd build
cmake .. -DBUILD_PYTHON=ON -DPYTHON_EXECUTABLE=$(which python3)
make
```

## :books: Example Usages

- [:croissant: C++](examples/cpp)
- [:snake: Python](examples/python)