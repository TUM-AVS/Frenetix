#ifndef TRAJECTORYSTRATEGYBINDING_HPP
#define TRAJECTORYSTRATEGYBINDING_HPP

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "strategies/TrajectoryStrategy.hpp"

//Submodule includes
#include "costStrategyBinding.hpp"
#include "feasabilityStrategyBinding.hpp"
#include "trajectoryFunctionsBinding/otherFunctionsBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindTrajectoryStrategy(nb::module_ &m) 
    {
        nb::module_ sub_m = m.def_submodule("trajectory_functions");

        nb::class_<TrajectoryStrategy>(sub_m, "TrajectoryStrategy");

        initBindCostStrategy(sub_m);
        initBindFeasabilityStrategy(sub_m);
        initBindOtherFunctions(sub_m);

    }


} //plannerCPP

#endif //TRAJECTORYSTRATEGYBINDING_HPP
