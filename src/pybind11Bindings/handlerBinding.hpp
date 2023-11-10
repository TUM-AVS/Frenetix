#pragma once

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "TrajectoryHandler.hpp"
#include "util.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindHandler(pybind11::module &m);

} //plannerCPP

