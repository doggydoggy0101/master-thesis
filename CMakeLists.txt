cmake_minimum_required(VERSION 3.11)
project(registration VERSION 1.0.0)
set(CMAKE_CXX_STANDARD 17)

option(BUILD_PYTHON "Build python wrappers" OFF)

set(CMAKE_BUILD_TYPE Release)
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

include(FetchContent)

# Eigen
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

# PMC
FetchContent_Declare(pmc
    GIT_REPOSITORY https://github.com/jingnanshi/pmc.git
    GIT_TAG        ebc296c2710f8ae9e89b7cc78d73ff6b1a651bd6
    SOURCE_DIR     "${CMAKE_SOURCE_DIR}/third-party/pmc"
)
FetchContent_MakeAvailable(pmc)


add_subdirectory(registration)
message(STATUS "Building source done")


if (BUILD_PYTHON)
    set(Python3_FIND_STRATEGY LOCATION)  # find active Python for venv
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
    message(STATUS "Using Python executable: ${PYTHON_EXECUTABLE}")

    # pybind11
    FetchContent_Declare(pybind11
        GIT_REPOSITORY https://github.com/pybind/pybind11.git
        GIT_TAG        v2.13
        SOURCE_DIR     "${CMAKE_SOURCE_DIR}/third-party/pybind"
    )
    FetchContent_MakeAvailable(pybind11)

    add_subdirectory(python)
    message(STATUS "Building python wrapper done")
endif()
