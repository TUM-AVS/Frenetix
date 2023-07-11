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
            .def_ro("is_initialized", &CurviLinearSample::isInitialized)
	    .def_ro("s", &CurviLinearSample::s)
	    .def_ro("s_dot", &CurviLinearSample::ss)
	    .def_ro("s_ddot", &CurviLinearSample::sss)
	    .def_ro("d", &CurviLinearSample::d)
	    .def_ro("d_dot", &CurviLinearSample::dd)
	    .def_ro("d_ddot", &CurviLinearSample::ddd)
	    .def_ro("theta", &CurviLinearSample::theta)
	    .def("__str__", [](const CurviLinearSample &cls) {
                                std::ostringstream oss;
                                cls.print(oss);
                                return oss.str();});

    }

} //plannerCPP


#endif //CURVILINEARSAMPLEBINDING_HPP
