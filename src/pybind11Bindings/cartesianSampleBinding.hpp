#ifndef CARTESIANSAMPLEBINDING_HPP
#define CARTESIANSAMPLEBINDING_HPP

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "trajectory/CartesianSample.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindCartesianSample(pybind11::module &m)
    {
        // Bind the CartesianSample class
        py::class_<CartesianSample>(m, "CartesianSample")
            .def(py::init<>())
            .def(py::init<const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&>(),
                py::arg("x"),
                py::arg("y"),
                py::arg("theta_gl"),
                py::arg("v"),
                py::arg("a"),
                py::arg("kappa_gl"),
                py::arg("kappa_dot"))
            .def_readwrite("is_initialized", &CartesianSample::isInitialized)
            .def_property("x",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd> { return self.x;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.x = arr;})
            .def_property("y",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.y;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.y = arr;})
            .def_property("theta",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.theta;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.theta = arr;})
            .def_property("v",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.velocity;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.velocity = arr;})
            .def_property("a",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.acceleration;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.acceleration = arr;})
            .def_property("kappa",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.kappa;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.kappa = arr;})
            .def_property("kappa_dot",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.kappaDot;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.kappaDot = arr;})
            .def("__str__", [](const CartesianSample &cs) {
                            std::ostringstream oss;
                            cs.print(oss);
                            return oss.str();});

    }

} //plannerCPP


#endif //CARTESIANSAMPLEBINDING_HPP
