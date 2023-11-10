#pragma once

//pybind includes
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "geometryMsgs.hpp"

namespace py = pybind11;

namespace plannerCPP
{
    void initBindGeometryMsg(pybind11::module &m);
}
