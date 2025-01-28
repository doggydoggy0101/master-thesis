/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <registration/mcis.h>
#include <registration/solver.h>
#include <registration/tuple.h>

namespace py = pybind11;

PYBIND11_MODULE(registration_python, module) {
  py::class_<registration::IrlsSolver>(module, "IrlsSolver")
      .def(py::init<const size_t, const double, std::string, double>())
      .def("solve", &registration::IrlsSolver::solve);

  py::class_<registration::GncSolver>(module, "GncSolver")
      .def(py::init<const size_t, const double, std::string, double, double, double>())
      .def("solve", &registration::GncSolver::solve);
  
  py::class_<registration::FracgmSolver>(module, "FracgmSolver")
      .def(py::init<const size_t, const double, double>())
      .def("solve", &registration::FracgmSolver::solve);

  py::module_ outlier_rejection = module.def_submodule("outlier_rejection");
  outlier_rejection
      .def("maximum_clique_inlier_selection", &registration::outlier_rejection::maximum_clique_inlier_selection)
      .def("tuple_test", &registration::outlier_rejection::tuple_test);
}