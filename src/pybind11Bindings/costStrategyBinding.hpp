#ifndef COSTSTRATEGYBINDING_HPP
#define COSTSTRATEGYBINDING_HPP

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "strategies/CostStrategy.hpp"
#include "trajectoryFunctionsBinding/costFunctionsBinding.hpp"

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

#endif //COSTSTRATEGYBINDING_HPP