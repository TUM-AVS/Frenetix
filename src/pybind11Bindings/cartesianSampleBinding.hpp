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
            .def(nb::init<const Eigen::Ref<Eigen::VectorXd>, 
                          const Eigen::Ref<Eigen::VectorXd>,
                          const Eigen::Ref<Eigen::VectorXd>, 
                          const Eigen::Ref<Eigen::VectorXd>,
                          const Eigen::Ref<Eigen::VectorXd>, 
                          const Eigen::Ref<Eigen::VectorXd>,
                          const Eigen::Ref<Eigen::VectorXd>>(),
                nb::arg("x"),
                nb::arg("y"),
                nb::arg("theta_gl"),
                nb::arg("v"),
                nb::arg("a"),
                nb::arg("kappa_gl"),
                nb::arg("kappa_dot"))
            .def_rw("is_initialized", &CartesianSample::isInitialized)
		/*
            .def_prop_rw("x",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd> { return self.x;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.x = arr;})
            .def_prop_rw("y",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.y;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.y = arr;})
            .def_prop_rw("theta",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.theta;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.theta = arr;})
            .def_prop_rw("v",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.velocity;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.velocity = arr;})
            .def_prop_rw("a",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.acceleration;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.acceleration = arr;})
            .def_prop_rw("kappa",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.kappa;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.kappa = arr;})
            .def_prop_rw("kappa_dot",
                          [](CartesianSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.kappaDot;},
                          [](CartesianSample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.kappaDot = arr;})
		*/
            .def("__str__", [](const CartesianSample &cs) {
                            std::ostringstream oss;
                            cs.print(oss);
                            return oss.str();});

    }

} //plannerCPP


#endif //CARTESIANSAMPLEBINDING_HPP
