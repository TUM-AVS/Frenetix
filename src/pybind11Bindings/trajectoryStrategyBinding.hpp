#pragma once

//pybind includes

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "strategies/TrajectoryStrategy.hpp"

//Submodule includes
#include "costStrategyBinding.hpp"
#include "feasabilityStrategyBinding.hpp"
#include "trajectoryFunctionsBinding/otherFunctionsBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindTrajectoryStrategy(nb::module_ &m);

} //plannerCPP

