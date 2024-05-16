//pybind includes
#include <nanobind/eigen/dense.h> // IWYU pragma: keep
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h> // IWYU pragma: keep
#include <nanobind/stl/string.h> // IWYU pragma: keep

#include <Eigen/Core>
#include <iosfwd>
#include <vector>

#include "geometryMsgs.hpp"

#include "geometryMsgsBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindGeometryMsg(nb::module_ &m)
    {

        nb::class_<PoseWithCovariance>(m, "PoseWithCovariance")
            .def("__init__", [] (PoseWithCovariance *pose, const Eigen::Vector3d& position, const Eigen::Vector4d& orientation, const Eigen::Matrix<double,6,6>& covariance) {
                new (pose) PoseWithCovariance(position, Eigen::Quaterniond(orientation), covariance);
            })
            .def("__repr__", [](const PoseWithCovariance &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_ro("position", &PoseWithCovariance::position)
            .def_prop_ro("orientation",
                          [](PoseWithCovariance &self) -> Eigen::Ref<Eigen::Vector4d> { return self.orientation.coeffs(); })
            .def_ro("covariance", &PoseWithCovariance::covariance);

        nb::class_<PredictedObject>(m, "PredictedObject")
            .def(nb::init<int, const std::vector<PoseWithCovariance>&, double, double>())
            .def("__repr__", [](const PredictedObject &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_ro("object_id", &PredictedObject::object_id)
            .def_ro("length", &PredictedObject::length)
            .def_ro("width", &PredictedObject::width)
            .def_ro("predictedPath", &PredictedObject::predictedPath);
    }
}
