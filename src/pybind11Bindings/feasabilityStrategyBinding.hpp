#pragma once

//pybind includes

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "strategies/FeasabilityStrategy.hpp"
#include "trajectoryFunctionsBinding/feasabilityFunctionsBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindFeasabilityStrategy(nb::module_ &m);

} //plannerCPP

