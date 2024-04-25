//pybind includes
#include <nanobind/nanobind.h>
#include <memory>

#include "strategies/CostStrategy.hpp"
#include "trajectoryFunctionsBinding/costFunctionsBinding.hpp"

#include "costStrategyBinding.hpp"

class TrajectoryStrategy;

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCostStrategy(nb::module_ &m) 
    {
        nb::class_<CostStrategy, TrajectoryStrategy>(m, "CostStrategy");
        nb::module_ sub_m = m.def_submodule("cost_functions");

        initBindCostFunctions(sub_m);
    }

} //plannerCPP

