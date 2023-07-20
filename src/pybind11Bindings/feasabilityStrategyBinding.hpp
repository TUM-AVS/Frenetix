#ifndef FEASABILITYSTRATEGYBINDING_HPP
#define FEASABILITYSTRATEGYBINDING_HPP

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "strategies/FeasabilityStrategy.hpp"
#include "trajectoryFunctionsBinding/feasabilityFunctionsBinding.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindFeasabilityStrategy(pybind11::module &m) 
    {
        py::class_<FeasabilityStrategy, TrajectoryStrategy, std::shared_ptr<FeasabilityStrategy>>(m, "FeasabilityStrategy");

        py::module sub_m = m.def_submodule("feasability_functions");

        initBindFeasabilityFunctions(sub_m);
    }

} //plannerCPP

#endif //FEASABILITYSTRATEGYBINDING_HPP