/**
 * Copyright 2025 Bang-Shien Chen.
 * All rights reserved. See LICENSE for the license information.
 */

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <registration/mcis.h>
#include <registration/parameter.h>
#include <registration/solver.h>
#include <registration/tuple.h>

namespace py = pybind11;

PYBIND11_MODULE(registration_python, module) {
  py::class_<registration::Params>(module, "Params")
      .def(py::init<>())
      .def_readwrite("max_iteration", &registration::Params::max_iteration)
      .def_readwrite("tolerance", &registration::Params::tolerance)
      .def_readwrite("robust_type", &registration::Params::robust_type)
      .def_readwrite("threshold_c", &registration::Params::threshold_c)
      .def_readwrite("gnc_factor", &registration::Params::gnc_factor)
      .def_readwrite("weight_tolerance", &registration::Params::weight_tolerance)
      .def_readwrite("majorization", &registration::Params::majorization)
      .def_readwrite("superlinear", &registration::Params::superlinear);

  py::class_<registration::IrlsSolver>(module, "IrlsSolver")
      .def(py::init<registration::Params>(), py::arg("params"))
      .def("solve", &registration::IrlsSolver::solve, py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound") = 0.1);

  py::class_<registration::GncSolver>(module, "GncSolver")
      .def(py::init<registration::Params>(), py::arg("params"))
      .def("solve", &registration::GncSolver::solve, py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound") = 0.1);

  py::class_<registration::FracgmSolver>(module, "FracgmSolver")
      .def(py::init<registration::Params>(), py::arg("params"))
      .def("solve", &registration::FracgmSolver::solve, py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound") = 0.1);

  py::module_ outlier_rejection = module.def_submodule("outlier_rejection");
  outlier_rejection
      .def("maximum_clique_inlier_selection", &registration::outlier_rejection::maximum_clique_inlier_selection,
           py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound"), py::arg("pmc_timeout") = 3600.0,
           py::arg("pmc_n_threads") = 4)
      .def("tuple_test", &registration::outlier_rejection::tuple_test, py::arg("pcd1"), py::arg("pcd2"),
           py::arg("tuple_scale") = 0.95, py::arg("max_tuple_count") = 1000);
}