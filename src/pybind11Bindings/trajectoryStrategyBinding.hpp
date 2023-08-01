#ifndef TRAJECTORYSTRATEGYBINDING_HPP
#define TRAJECTORYSTRATEGYBINDING_HPP

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

    void initBindTrajectoryStrategy(pybind11::module &m) 
    {
        py::module sub_m = m.def_submodule("trajectory_functions");

        py::class_<TrajectoryStrategy, std::shared_ptr<TrajectoryStrategy>>(sub_m, "TrajectoryStrategy")
            .def("evaluate_trajectory", &TrajectoryStrategy::evaluateTrajectory, py::arg("trajectory"))
            .def_property_readonly("name", &TrajectoryStrategy::getFunctionName);

        initBindCostStrategy(sub_m);
        initBindFeasabilityStrategy(sub_m);
        initBindOtherFunctions(sub_m);

    }


} //plannerCPP

#endif //TRAJECTORYSTRATEGYBINDING_HPP