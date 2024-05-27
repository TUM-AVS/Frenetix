#pragma once

//pybind includes

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "trajectory/CurvilinearSample.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCurviLinearSample(nb::module_ &m);

} //plannerCPP

