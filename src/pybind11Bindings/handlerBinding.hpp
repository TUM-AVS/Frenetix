#ifndef HANDLERBINDING_HPP
#define HANDLERBINDING_HPP

//pybind includes
#include <nanobind/nanobind.h>
// #include <nanobind/ndarray.h>
// #include <nanobind/stl/bind_vector.h>
#include <nanobind/stl/map.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/eigen/dense.h>
#include <nanobind/make_iterator.h>

#include "TrajectoryHandler.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindHandler(nb::module_ &m)
    {
        nb::class_<TrajectoryHandler>(m, "TrajectoryHandler")
            .def(nb::init<double>(), nb::arg("dt"))
            .def
            (
                "generate_trajectories",
                [](TrajectoryHandler &self, const nb::DRef<SamplingMatrixXd>& samplingMatrix, bool lowVelocityMode)
                {
                    self.generateTrajectories(samplingMatrix, lowVelocityMode);
                },
                nb::arg("samplingMatrix"),
                nb::arg("lowVelocityMode")
            )
            .def("sort", &TrajectoryHandler::sort)
            .def
            (
                "add_feasability_function",
                [](TrajectoryHandler &self, std::shared_ptr<FeasabilityStrategy> function)
                {
                    self.addFeasabilityFunction(function);
                },
                nb::arg().noconvert()
            )
            .def
            (
                "add_cost_function",
                [](TrajectoryHandler &self, std::shared_ptr<CostStrategy> function)
                {
                    self.addCostFunction(function);
                },
                nb::arg().noconvert()
            )
            .def
            (
                "add_function",
                [](TrajectoryHandler &self, std::shared_ptr<TrajectoryStrategy> function)
                {
                    self.addFunction(function);
                },
                nb::arg().noconvert()
            )
            .def
            (
                "evaluate_all_current_functions",
                &TrajectoryHandler::evaluateAllCurrentFunctions,
                nb::arg("calculateAllCosts") = false
            )
            .def
            (
                "evaluate_all_current_functions_concurrent",
                &TrajectoryHandler::evaluateAllCurrentFunctionsConcurrent,
                nb::arg("calculateAllCosts") = false
            )
            .def
            (
                "get_sorted_trajectories",
                [](TrajectoryHandler &self)
                {
                    self.sort();
                    return self.m_trajectories;
                    // return self
                    // return nb::make_iterator(
                    //     nb::type<TrajectorySample>(), "iterator",
                    // self.m_trajectories.begin(), self.m_trajectories.end());
                }
                // , nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            /* .def
            (
                "get_cost_functions",
                [](TrajectoryHandler &self)
                {
                    return nb::make_iterator(self.m_costFunctions.begin(), self.m_costFunctions.end());
                },
                nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "get_feasability_functions",
                [](TrajectoryHandler &self)
                {
                    return nb::make_iterator(self.m_feasabilityFunctions.begin(), self.m_feasabilityFunctions.end());
                },
                nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "get_other_functions",
                [](TrajectoryHandler &self)
                {
                return nb::make_iterator(self.m_otherFunctions.begin(), self.m_otherFunctions.end());
                },
                nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            ) */
            .def
            (
                "reset_Trajectories", 
                &TrajectoryHandler::resetTrajectories, 
                "Resets the trajectories container."
            );
    }

} //plannerCPP


#endif //HANDLERBINDING_HPP
