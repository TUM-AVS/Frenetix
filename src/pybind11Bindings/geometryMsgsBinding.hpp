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
        nb::class_<Position>(m, "Position")
            .def(nb::init<>())
            .def_rw("x", &Position::x)
            .def_rw("y", &Position::y)
            .def_rw("z", &Position::z);

        nb::class_<Orientation>(m, "Orientation")
            .def(nb::init<>())
            .def_rw("x", &Orientation::x)
            .def_rw("y", &Orientation::y)
            .def_rw("z", &Orientation::z)
            .def_rw("w", &Orientation::w);

        nb::class_<Pose>(m, "Pose")
            .def(nb::init<>())
            .def_rw("position", &Pose::position)
            .def_rw("orientation", &Pose::orientation);

        nb::class_<PoseWithCovariance>(m, "PoseWithCovariance")
            .def(nb::init<>())
            .def_rw("pose", &PoseWithCovariance::pose)
            .def_rw("covariance", &PoseWithCovariance::covariance);
    }
}