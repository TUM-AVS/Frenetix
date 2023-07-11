#ifndef CARTESIANSAMPLEBINDING_HPP
#define CARTESIANSAMPLEBINDING_HPP

//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "trajectory/CartesianSample.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCartesianSample(nb::module_ &m) 
    {
        // Bind the CartesianSample class
        nb::class_<CartesianSample>(m, "CartesianSample")
            .def(nb::init<>())
            .def(nb::init<const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&>(),
                nb::arg("x"),
                nb::arg("y"),
                nb::arg("theta_gl"),
                nb::arg("v"),
                nb::arg("a"),
                nb::arg("kappa_gl"),
                nb::arg("kappa_dot"))
            .def_rw("is_initialized", &CartesianSample::isInitialized)
            .def_ro("x", &CartesianSample::x)
            .def_ro("y", &CartesianSample::y)
            .def_ro("theta", &CartesianSample::theta)
            .def_ro("v", &CartesianSample::velocity)
            .def_ro("a", &CartesianSample::acceleration)
            .def_ro("kappa", &CartesianSample::kappa)
            .def_ro("kappa_dot", &CartesianSample::kappaDot)
            .def("__str__", [](const CartesianSample &cs) {
                            std::ostringstream oss;
                            cs.print(oss);
                            return oss.str();});

    }

} //plannerCPP


#endif //CARTESIANSAMPLEBINDING_HPP
