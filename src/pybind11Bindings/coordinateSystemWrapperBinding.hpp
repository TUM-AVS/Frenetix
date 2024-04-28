#pragma once

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "CoordinateSystemWrapper.hpp"
#include "geometry/curvilinear_coordinate_system.h"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCoordinateSystemWrapper(nb::module_ &m);
} //plannerCPP

