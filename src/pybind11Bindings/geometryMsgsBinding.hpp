#pragma once

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "geometryMsgs.hpp"

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindGeometryMsg(nb::module_ &m);
}
