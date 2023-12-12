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
            // .def(py::init<>())
            .def(py::init<const Eigen::Vector3d&, const Eigen::Vector4d&, const Eigen::Matrix<double,6,6>&>())
            .def("__repr__", [](const PoseWithCovariance &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_readonly("position", &PoseWithCovariance::position)
            .def_readonly("orientation", &PoseWithCovariance::orientation)
            .def_readonly("covariance", &PoseWithCovariance::covariance)
            .def(py::pickle(
                [](const PoseWithCovariance &pose) { // __getstate__
                    using namespace pybind11::literals; // to bring in the `_a` literal

                    py::dict d(
                        "position"_a=pose.position,
                        "orientation"_a=pose.orientation,
                        "covariance"_a=pose.covariance
                    );

                    return d;
                },
                [](py::dict d) { // __setstate__
                    auto position = d["position"].cast<Eigen::Vector3d>();
                    auto orientation = d["orientation"].cast<Eigen::Vector4d>();
                    auto covariance = d["covariance"].cast<Eigen::Matrix<double,6,6>>();

                    return PoseWithCovariance(position, orientation, covariance);
                }
            ));

        py::class_<PredictedObject>(m, "PredictedObject")
            //.def(py::init<size_t>())
            .def(py::init<int, const std::vector<PoseWithCovariance>&>())
            .def("__repr__", [](const PredictedObject &p) {
                std::ostringstream oss;
                oss << p;
                return oss.str();
            })
            .def_readonly("object_id", &PredictedObject::object_id)
            //.def_readonly("length", &PredictedObject::length)
            //.def_readonly("width", &PredictedObject::width)
            .def_readonly("predictedPath", &PredictedObject::predictedPath)
            .def(py::pickle(
                [](const PredictedObject &p) { // __getstate__
                    using namespace pybind11::literals; // to bring in the `_a` literal

                    py::dict d(
                        "object_id"_a=p.object_id,
                        "predicted_path"_a=p.predictedPath
                    );

                    return d;
                },
                [](py::dict d) { // __setstate__
                    auto object_id = d["object_id"].cast<int>();
                    auto predictedPath = d["predicted_path"].cast<std::vector<PoseWithCovariance>>();

                    return PredictedObject(object_id, predictedPath);
                }
            ));
    }
}
