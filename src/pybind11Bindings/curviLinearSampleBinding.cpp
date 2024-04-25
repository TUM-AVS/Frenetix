//pybind includes
#include <nanobind/eigen/dense.h> // IWYU pragma: keep
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <Eigen/Core>
#include <iosfwd>

#include "trajectory/CurvilinearSample.hpp"

#include "curviLinearSampleBinding.hpp"

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
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd> { return self.s;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.s = arr;})
            .def_prop_rw("d",                               
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.d;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.d = arr;})
            .def_prop_rw("theta",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.theta;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.theta = arr;})
            .def_prop_rw("d_dot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.dd;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.dd = arr;})
            .def_prop_rw("d_ddot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.ddd;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.ddd = arr;})
            .def_prop_rw("s_dot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.ss;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.ss = arr;})
            .def_prop_rw("s_ddot",
                          [](CurviLinearSample &self) -> Eigen::Ref<Eigen::VectorXd>{ return self.sss;},
                          [](CurviLinearSample &self, const Eigen::Ref<const Eigen::VectorXd>& arr) {self.sss = arr;})
            .def("__str__", [](const CurviLinearSample &cls) {
                                std::ostringstream oss;
                                cls.print(oss);
                                return oss.str();})
            .def("__getstate__", 
                [](const CurviLinearSample &curv) {
                    nb::dict d;
                        d["s"]=curv.s;
                        d["d"]=curv.d;
                        d["theta"]=curv.theta;
                        d["d_dot"]=curv.dd;
                        d["d_ddot"]=curv.ddd;
                        d["s_dot"]=curv.ss;
                        d["s_ddot"]=curv.sss;

                    return d;
             })
	    .def("__setstate__", 
                [](CurviLinearSample &cls, nb::dict d) {
                    new (&cls) CurviLinearSample  {
			nb::cast<Eigen::Ref<Eigen::VectorXd>>(d["s"]),
                        nb::cast<Eigen::Ref<Eigen::VectorXd>>(d["d"]),
                        nb::cast<Eigen::Ref<Eigen::VectorXd>>(d["theta"]),
                        nb::cast<Eigen::Ref<Eigen::VectorXd>>(d["d_dot"]),
                        nb::cast<Eigen::Ref<Eigen::VectorXd>>(d["d_ddot"]),
                        nb::cast<Eigen::Ref<Eigen::VectorXd>>(d["s_dot"]),
                        nb::cast<Eigen::Ref<Eigen::VectorXd>>(d["s_ddot"])
                    };
                }
            )
    ;

    }

} //plannerCPP

