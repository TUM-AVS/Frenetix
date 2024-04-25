//pybind includes
#include <nanobind/nanobind.h>
#include <memory>

#include "strategies/FeasabilityStrategy.hpp"
#include "trajectoryFunctionsBinding/feasabilityFunctionsBinding.hpp"

#include "feasabilityStrategyBinding.hpp"

class TrajectoryStrategy;

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindFeasabilityStrategy(nb::module_ &m) 
    {
        nb::class_<FeasabilityStrategy, TrajectoryStrategy>(m, "FeasabilityStrategy");

        nb::module_ sub_m = m.def_submodule("feasability_functions");

        initBindFeasabilityFunctions(sub_m);
    }

} //plannerCPP

