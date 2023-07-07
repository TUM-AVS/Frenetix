#ifndef FEASABILITYSTRATEGYBINDING_HPP
#define FEASABILITYSTRATEGYBINDING_HPP

//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>
#include <string>

#include "strategies/FeasabilityStrategy.hpp"
#include "trajectoryFunctionsBinding/feasabilityFunctionsBinding.hpp"

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

#endif //FEASABILITYSTRATEGYBINDING_HPP
