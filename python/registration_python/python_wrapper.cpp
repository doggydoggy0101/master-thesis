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
      .def(py::init<const size_t, const double, const std::string&, const double>(), py::arg("max_iteration") = 1000,
           py::arg("tolerance") = 1e-6, py::arg("robust_type") = "GM", py::arg("threshold_c") = 1.0)
      .def("solve", &registration::IrlsSolver::solve, py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound") = 0.1);

  py::class_<registration::GncSolver>(module, "GncSolver")
      .def(py::init<const size_t, const double, const std::string&, const double, const double, const double>(),
           py::arg("max_iteration") = 1000, py::arg("tolerance") = 1e-6, py::arg("robust_type") = "GM",
           py::arg("threshold_c") = 1.0, py::arg("gnc_factor") = 1.4, py::arg("weight_tolerance") = 1e-4)
      .def("solve", &registration::GncSolver::solve, py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound") = 0.1);

  py::class_<registration::FracgmSolver>(module, "FracgmSolver")
      .def(py::init<const size_t, const double, const double>(), py::arg("max_iteration") = 1000,
           py::arg("tolerance") = 1e-6, py::arg("threshold_c") = 1.0)
      .def("solve", &registration::FracgmSolver::solve, py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound") = 0.1);

  py::module_ outlier_rejection = module.def_submodule("outlier_rejection");
  outlier_rejection
      .def("maximum_clique_inlier_selection", &registration::outlier_rejection::maximum_clique_inlier_selection,
           py::arg("pcd1"), py::arg("pcd2"), py::arg("noise_bound"), py::arg("pmc_timeout") = 3600.0,
           py::arg("pmc_n_threads") = 4)
      .def("tuple_test", &registration::outlier_rejection::tuple_test, py::arg("pcd1"), py::arg("pcd2"),
           py::arg("tuple_scale") = 0.95, py::arg("max_tuple_count") = 1000);
}