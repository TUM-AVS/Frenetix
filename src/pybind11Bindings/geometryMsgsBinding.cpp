//pybind includes
#include <pybind11/eigen.h> // IWYU pragma: keep
#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // IWYU pragma: keep
#include <Eigen/Core>
#include <iosfwd>
#include <vector>

#include "geometryMsgs.hpp"

#include "geometryMsgsBinding.hpp"

namespace py = pybind11;

namespace plannerCPP
{
    void initBindGeometryMsg(pybind11::module &m)
    {

        py::class_<PoseWithCovariance>(m, "PoseWithCovariance")
            .def(py::init([] (const Eigen::Vector3d& position, const Eigen::Vector4d& orientation, const Eigen::Matrix<double,6,6>& covariance) {
                return std::make_unique<PoseWithCovariance>(position, Eigen::Quaterniond(orientation), covariance);
            }))
            .def("__repr__", [](const PoseWithCovariance &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_readonly("position", &PoseWithCovariance::position)
            .def_property_readonly("orientation",
                          [](PoseWithCovariance &self) -> Eigen::Ref<Eigen::Vector4d> { return self.orientation.coeffs(); })
            .def_readonly("covariance", &PoseWithCovariance::covariance);

        py::class_<PredictedObject>(m, "PredictedObject")
            //.def(py::init<size_t>())
            .def(py::init<int, const std::vector<PoseWithCovariance>&, double, double>())
            .def("__repr__", [](const PredictedObject &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_readonly("object_id", &PredictedObject::object_id)
            .def_readonly("length", &PredictedObject::length)
            .def_readonly("width", &PredictedObject::width)
            .def_readonly("predictedPath", &PredictedObject::predictedPath);
    }
}
