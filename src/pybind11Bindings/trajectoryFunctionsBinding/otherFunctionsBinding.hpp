#pragma once

//pybind includes

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindOtherFunctions(nb::module_ &m);
} //plannerCPP

