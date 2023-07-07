#ifndef COORDINATESYSTEMWRAPPERBINDING_HPP
#define COORDINATESYSTEMWRAPPERBINDING_HPP

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "CoordinateSystemWrapper.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCoordinateSystemWrapper(nb::module_ &m)
    {
        nb::class_<CoordinateSystemWrapper>(m, "CoordinateSystemWrapper")
            .def(nb::init<Eigen::Ref<RowMatrixXd>>(), nb::arg("ref_path"))
            .def_prop_rw("system", &CoordinateSystemWrapper::getSystem, &CoordinateSystemWrapper::setSystem)
            .def_prop_rw("ref_pos",
                          [](CoordinateSystemWrapper &self) -> Eigen::VectorXd { return self.m_refPos;},
                          [](CoordinateSystemWrapper &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.m_refPos = arr;})
            .def_prop_rw("ref_curv",
                          [](CoordinateSystemWrapper &self) -> Eigen::VectorXd { return self.m_refCurv;},
                          [](CoordinateSystemWrapper &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.m_refCurv = arr;})
            .def_prop_rw("ref_theta",
                          [](CoordinateSystemWrapper &self) -> Eigen::VectorXd { return self.m_refTheta;},
                          [](CoordinateSystemWrapper &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.m_refTheta = arr;})
            .def_prop_rw("ref_curv_d",
                          [](CoordinateSystemWrapper &self) -> Eigen::VectorXd { return self.m_refCurvD;},
                          [](CoordinateSystemWrapper &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.m_refCurvD = arr;})
            .def_prop_rw("ref_curv_dd",
                          [](CoordinateSystemWrapper &self) -> Eigen::VectorXd { return self.m_refCurvDD;},
                          [](CoordinateSystemWrapper &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.m_refCurvDD = arr;})
            .def_rw("ref_line", &CoordinateSystemWrapper::m_refPolyLineFromCoordSys);

    }
} //plannerCPP

#endif //COORDINATESYSTEMWRAPPERBINDING_HPP
