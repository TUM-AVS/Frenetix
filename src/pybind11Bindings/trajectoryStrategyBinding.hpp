#pragma once

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "strategies/TrajectoryStrategy.hpp"

//Submodule includes
#include "costStrategyBinding.hpp"
#include "feasabilityStrategyBinding.hpp"
#include "trajectoryFunctionsBinding/otherFunctionsBinding.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindTrajectoryStrategy(pybind11::module &m);

} //plannerCPP

