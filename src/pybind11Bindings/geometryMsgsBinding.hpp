//pybind includes
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "geometryMsgs.hpp"

namespace py = pybind11;

namespace plannerCPP
{
    void initBindGeometryMsg(pybind11::module &m)
    {

        py::class_<PoseWithCovariance>(m, "PoseWithCovariance")
            // .def(py::init<>())
            .def(py::init<const Eigen::Vector3d&, const Eigen::Vector4d&, const Eigen::Matrix<double,6,6>&>())
            .def("__repr__", [](const PoseWithCovariance &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_readonly("position", &PoseWithCovariance::position)
            .def_readonly("orientation", &PoseWithCovariance::orientation)
            .def_readonly("covariance", &PoseWithCovariance::covariance);

        py::class_<PredictedObject>(m, "PredictedObject")
            //.def(py::init<size_t>())
            .def(py::init<int, const std::vector<PoseWithCovariance>&>())
            .def("__repr__", [](const PredictedObject &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_readonly("object_id", &PredictedObject::object_id)
            .def_readonly("predictedPath", &PredictedObject::predictedPath);
    }
}
