//pybind includes
#include <pybind11/eigen.h> // IWYU pragma: keep
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include <iosfwd>

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
                            return oss.str();}
	        )
            .def(py::pickle(
                [](const CartesianSample &cart) { // __getstate__
                    using namespace pybind11::literals; // to bring in the `_a` literal

                    py::dict d(
                        "x"_a=cart.x,
                        "y"_a=cart.y,
                        "theta"_a=cart.theta,
                        "v"_a=cart.velocity,
                        "a"_a=cart.acceleration,
                        "kappa"_a=cart.kappa,
                        "kappa_dot"_a=cart.kappaDot
                    );

                    return d;
                },
                [](py::dict d) { // __setstate__
                    CartesianSample cart {
                        d["x"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["y"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["theta"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["v"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["a"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["kappa"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["kappa_dot"].cast<Eigen::Ref<Eigen::VectorXd>>()
                    };

                    return cart;
                }
            ));

    }

} //plannerCPP

