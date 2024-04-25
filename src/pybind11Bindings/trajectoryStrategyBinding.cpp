//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <memory>

//Submodule includes
#include "costStrategyBinding.hpp"
#include "feasabilityStrategyBinding.hpp"
#include "strategies/TrajectoryStrategy.hpp"
#include "trajectoryFunctionsBinding/otherFunctionsBinding.hpp"
#include "trajectory/TrajectorySample.hpp"

#include "trajectoryStrategyBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindTrajectoryStrategy(nb::module_ &m) 
    {
        nb::module_ sub_m = m.def_submodule("trajectory_functions");

        nb::class_<TrajectoryStrategy>(sub_m, "TrajectoryStrategy")
            .def("evaluate_trajectory", &TrajectoryStrategy::evaluateTrajectory, nb::arg("trajectory"))
            .def_prop_ro("name", &TrajectoryStrategy::getFunctionName);

        initBindCostStrategy(sub_m);
        initBindFeasabilityStrategy(sub_m);
        initBindOtherFunctions(sub_m);

    }


} //plannerCPP

