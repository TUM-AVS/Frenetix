#pragma once

//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "TrajectoryHandler.hpp"
#include "util.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindHandler(nb::module_ &m);

} //plannerCPP

