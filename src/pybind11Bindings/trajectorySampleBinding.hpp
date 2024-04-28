#pragma once

//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "TrajectoryHandler.hpp"
#include "TrajectorySample.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindTrajectorySample(nb::module_ &m);

} //plannerCPP

