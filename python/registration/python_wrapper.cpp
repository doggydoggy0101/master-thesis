/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
// #include <registration/mcis.h>
#include <registration/fracgm.h>
#include <registration/qgm.h>

namespace py = pybind11;

PYBIND11_MODULE(qgm_python, module) {
  py::class_<registration::fracgm>(module, "fracgm")
      .def(py::init<const size_t, const double, const double, const double>())
      .def("solve", &registration::fracgm::solve);

  py::class_<registration::qgm>(module, "qgm")
      .def(py::init<const size_t, const double, const double, const double>())
      .def("solve", &registration::qgm::solve);

//   py::module_ mcis = module.def_submodule("mcis");
//   mcis.def("inlier_selection", &registration::mcis::inlier_selection);
}