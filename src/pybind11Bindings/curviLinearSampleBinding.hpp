#ifndef CURVILINEARSAMPLEBINDING_HPP
#define CURVILINEARSAMPLEBINDING_HPP

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "trajectory/CurvilinearSample.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCurviLinearSample(nb::module_ &m) 
    {
        // Bind the CurvilinearSample class
        nb::class_<CurviLinearSample>(m, "CurviLinearSample")
            .def(nb::init<>())
            .def(nb::init<const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&>(),
                nb::arg("s"),
                nb::arg("d"),
                nb::arg("theta_gl"),
                nb::arg("dd"),
                nb::arg("ddd"),
                nb::arg("ss"),
                nb::arg("sss"))
            .def_rw("is_initialized", &CurviLinearSample::isInitialized)
            .def_prop_rw("s",
                          [](CurviLinearSample &self) -> Eigen::VectorXd { return self.s;},
                          [](CurviLinearSample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.s = arr;})
            .def_prop_rw("d",
                          [](CurviLinearSample &self) -> Eigen::VectorXd { return self.d;},
                          [](CurviLinearSample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.d = arr;})
            .def_prop_rw("theta",
                          [](CurviLinearSample &self) -> Eigen::VectorXd { return self.theta;},
                          [](CurviLinearSample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.theta = arr;})
            .def_prop_rw("d_dot",
                          [](CurviLinearSample &self) -> Eigen::VectorXd { return self.dd;},
                          [](CurviLinearSample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.dd = arr;})
            .def_prop_rw("d_ddot",
                          [](CurviLinearSample &self) -> Eigen::VectorXd { return self.ddd;},
                          [](CurviLinearSample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.ddd = arr;})
            .def_prop_rw("s_dot",
                          [](CurviLinearSample &self) -> Eigen::VectorXd { return self.ss;},
                          [](CurviLinearSample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.ss = arr;})
            .def_prop_rw("s_ddot",
                          [](CurviLinearSample &self) -> Eigen::VectorXd { return self.sss;},
                          [](CurviLinearSample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.sss = arr;})
            .def("__str__", [](const CurviLinearSample &cls) {
                                std::ostringstream oss;
                                cls.print(oss);
                                return oss.str();});

    }

} //plannerCPP


#endif //CURVILINEARSAMPLEBINDING_HPP
