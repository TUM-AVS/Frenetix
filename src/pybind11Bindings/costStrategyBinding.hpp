#pragma once

//pybind includes

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "strategies/CostStrategy.hpp"
#include "trajectoryFunctionsBinding/costFunctionsBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCostStrategy(nb::module_ &m);

} //plannerCPP

