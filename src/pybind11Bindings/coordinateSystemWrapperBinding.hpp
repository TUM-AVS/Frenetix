#pragma once

//pybind includes
#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "CoordinateSystemWrapper.hpp"
#include "geometry/curvilinear_coordinate_system.h"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindCoordinateSystemWrapper(pybind11::module &m);
} //plannerCPP

