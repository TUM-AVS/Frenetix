//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h> // IWYU pragma: keep
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/unique_ptr.h>
#include <nanobind/stl/vector.h>

#include <Eigen/Core>
#include <memory>

#include "CoordinateSystemWrapper.hpp"
#include "util.hpp"

#include "coordinateSystemWrapperBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCoordinateSystemWrapper(nb::module_ &m)
    {
        // TODO: Registering the CCS class is required for correct type signatures,
        // but we can't register the class here since it is also exported by commonroad_dc.
        // Registering it multiple times would result in pybind11 errors.
        //nb::class_<geometry::CurvilinearCoordinateSystem>(m, "_CurvilinearCoordinateSystem")
        //    .def(nb::init<const geometry::EigenPolyline&, double, double, double>()); //, nb::module_local());

        nb::class_<CoordinateSystemWrapper>(m, "CoordinateSystemWrapper")
            .def(nb::init<Eigen::Ref<RowMatrixXd>>(), nb::arg("ref_path"))
            // CCS property is problematic...
            // .def_prop_rw("system", &CoordinateSystemWrapper::getSystem, &CoordinateSystemWrapper::setSystem)
            .def_prop_rw("ref_pos",
                          [](CoordinateSystemWrapper &self) -> Eigen::Ref<Eigen::VectorXd> { return self.m_refPos;},
                          [](CoordinateSystemWrapper &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.m_refPos = arr;})
            .def_prop_rw("ref_curv",
                          [](CoordinateSystemWrapper &self) -> Eigen::Ref<Eigen::VectorXd> { return self.m_refCurv;},
                          [](CoordinateSystemWrapper &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.m_refCurv = arr;})
            .def_prop_rw("ref_theta",
                          [](CoordinateSystemWrapper &self) -> Eigen::Ref<Eigen::VectorXd> { return self.m_refTheta;},
                          [](CoordinateSystemWrapper &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.m_refTheta = arr;})
            .def_prop_rw("ref_curv_d",
                          [](CoordinateSystemWrapper &self) -> Eigen::Ref<Eigen::VectorXd> { return self.m_refCurvD;},
                          [](CoordinateSystemWrapper &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.m_refCurvD = arr;})
            .def_prop_rw("ref_curv_dd",
                          [](CoordinateSystemWrapper &self) -> Eigen::Ref<Eigen::VectorXd> { return self.m_refCurvDD;},
                          [](CoordinateSystemWrapper &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.m_refCurvDD = arr;})
            .def_rw("ref_line", &CoordinateSystemWrapper::m_refPolyLineFromCoordSys)
            .def("__getstate__", 
                [](const CoordinateSystemWrapper &ccs) {
                    nb::dict d;
                    d["ref_path"] = ccs.getRefPath();

                    return d;
                }
                 )
            .def("__setstate__",
                [](CoordinateSystemWrapper &ccs, nb::dict d) {
                    RowMatrixXd refPath = nb::cast<RowMatrixXd>(d["ref_path"]);

                    new (&ccs) CoordinateSystemWrapper { refPath };
                }
            )
        
    ;

    }
} //plannerCPP

