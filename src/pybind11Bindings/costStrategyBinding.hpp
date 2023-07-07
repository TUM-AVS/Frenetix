#ifndef COSTSTRATEGYBINDING_HPP
#define COSTSTRATEGYBINDING_HPP

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "strategies/CostStrategy.hpp"
#include "trajectoryFunctionsBinding/costFunctionsBinding.hpp"

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

#endif //COSTSTRATEGYBINDING_HPP
