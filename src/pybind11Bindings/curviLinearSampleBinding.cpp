//pybind includes
#include <pybind11/eigen.h> // IWYU pragma: keep
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include <iosfwd>

#include "trajectory/CurvilinearSample.hpp"

#include "curviLinearSampleBinding.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindCurviLinearSample(pybind11::module &m) 
    {
        // Bind the CurvilinearSample class
        py::class_<CurviLinearSample>(m, "CurviLinearSample")
            .def(py::init<>())
            .def(py::init<const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&,
                          const Eigen::Ref<Eigen::VectorXd>&>(),
                py::arg("s"),
                py::arg("d"),
                py::arg("theta_gl"),
                py::arg("dd"),
                py::arg("ddd"),
                py::arg("ss"),
                py::arg("sss"))
            .def_readwrite("is_initialized", &CurviLinearSample::isInitialized)
            .def_property("s",                               
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd> { return self.s;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.s = arr;})
            .def_property("d",                               
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.d;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.d = arr;})
            .def_property("theta",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.theta;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.theta = arr;})
            .def_property("d_dot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.dd;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.dd = arr;})
            .def_property("d_ddot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.ddd;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.ddd = arr;})
            .def_property("s_dot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.ss;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.ss = arr;})
            .def_property("s_ddot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.sss;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.sss = arr;})
            .def("__str__", [](const CurviLinearSample &cls) {
                                std::ostringstream oss;
                                cls.print(oss);
                                return oss.str();})
            .def(py::pickle(
                [](const CurviLinearSample &curv) { // __getstate__
                    using namespace pybind11::literals; // to bring in the `_a` literal

                    py::dict d(
                        "s"_a=curv.s,
                        "d"_a=curv.d,
                        "theta"_a=curv.theta,
                        "d_dot"_a=curv.dd,
                        "d_ddot"_a=curv.ddd,
                        "s_dot"_a=curv.ss,
                        "s_ddot"_a=curv.sss
                    );

                    return d;
                },
                [](py::dict d) { // __setstate__
                    CurviLinearSample cart {
                        d["s"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["d"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["theta"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["d_dot"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["d_ddot"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["s_dot"].cast<Eigen::Ref<Eigen::VectorXd>>(),
                        d["s_ddot"].cast<Eigen::Ref<Eigen::VectorXd>>()
                    };

                    return cart;
                }
            ));

    }

} //plannerCPP

