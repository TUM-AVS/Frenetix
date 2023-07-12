//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h>

#include "geometryMsgs.hpp"

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindGeometryMsg(nb::module_ &m) 
    {

        nb::class_<PoseWithCovariance>(m, "PoseWithCovariance")
            .def(nb::init<>())
            .def("__repr__", [](const PoseWithCovariance &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_rw("position", &PoseWithCovariance::position)
            .def_rw("orientation", &PoseWithCovariance::orientation)
            .def_rw("covariance", &PoseWithCovariance::covariance);

        nb::class_<PredictedObject>(m, "PredictedObject")
            .def(nb::init<size_t>())
            .def("__repr__", [](const PredictedObject &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_rw("object_id", &PredictedObject::object_id)
            .def_rw("predictedPath", &PredictedObject::predictedPath);
    }
}