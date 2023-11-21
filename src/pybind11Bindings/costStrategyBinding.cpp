//pybind includes
#include <pybind11/pybind11.h>
#include <memory>

#include "strategies/CostStrategy.hpp"
#include "trajectoryFunctionsBinding/costFunctionsBinding.hpp"

#include "costStrategyBinding.hpp"

class TrajectoryStrategy;

namespace py = pybind11;

namespace plannerCPP
{

    void initBindCostStrategy(pybind11::module &m) 
    {
        py::class_<CostStrategy, TrajectoryStrategy, std::shared_ptr<CostStrategy>>(m, "CostStrategy");
        py::module sub_m = m.def_submodule("cost_functions");

        initBindCostFunctions(sub_m);
    }

} //plannerCPP

