/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <registration/mcis.h>
#include <registration/solver.h>

namespace py = pybind11;

PYBIND11_MODULE(registration_python, module) {
  py::class_<registration::FracGM>(module, "FracGM")
      .def(py::init<const size_t, const double, const double>())
      .def("solve", &registration::FracGM::solve);

  py::class_<registration::QGM>(module, "QGM")
      .def(py::init<const size_t, const double, const double>())
      .def("solve", &registration::QGM::solve);

  py::module_ outlier_rejection = module.def_submodule("outlier_rejection");
  outlier_rejection.def("maximum_clique_inlier_selection",
                        &registration::outlier_rejection::maximum_clique_inlier_selection);
}