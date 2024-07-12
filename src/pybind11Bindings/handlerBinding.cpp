//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/eigen/dense.h> // IWYU pragma: keep
#include <nanobind/make_iterator.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <Eigen/Core>
#include <map>
#include <memory>
#include <vector>

#include "TrajectoryHandler.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"
#include "TrajectoryStrategy.hpp"
#include "FeasabilityStrategy.hpp"
#include "CostStrategy.hpp"

#include "handlerBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindHandler(nb::module_ &m) 
    {
        nb::class_<SamplingConfiguration>(m, "SamplingConfiguration")
            .def(nb::init<double, double, double, double, int, bool, bool, bool>(),
                 // Requires nanobind >= 2.0
                 #if NB_VERSION_MAJOR >= 2
                 nb::kw_only(),
                 #endif
                 nb::arg("t_min"),
                 nb::arg("t_max"),
                 nb::arg("dt"),
                 nb::arg("d_delta"),
                 nb::arg("sampling_level"),
                 nb::arg("time_based_lateral_delta_scaling") = true,
                 nb::arg("enforce_time_bounds") = true,
                 nb::arg("strict_velocity_sampling") = true
            );

        nb::class_<TrajectoryHandler>(m, "TrajectoryHandler")
            .def(nb::init<double>(), nb::arg("dt"))
            .def("generate_stopping_trajectories", &TrajectoryHandler::generateStoppingTrajectories)
            .def
            (
                "generate_trajectories", 
                [](TrajectoryHandler &self, const Eigen::Ref<RowMatrixXd> samplingMatrix, bool lowVelocityMode)
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
            .def("clear_cost_functions", &TrajectoryHandler::clearCostFunctions, "Clears all cost functions.")
            .def("set_all_cost_weights_to_zero", &TrajectoryHandler::setAllCostWeightsToZero, "Sets all cost function weights to zero.")
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
                nb::arg("calculateAllCosts") = false,
                nb::call_guard<nb::gil_scoped_release>()
            )
            .def
            (
                "evaluate_all_current_functions_concurrent", 
                &TrajectoryHandler::evaluateAllCurrentFunctionsConcurrent,
                nb::arg("calculateAllCosts") = false,
                nb::call_guard<nb::gil_scoped_release>()
            )
            .def
            (
                "get_feasible_count",
                &TrajectoryHandler::getFeasibleCount
            )
            .def
            (
                "get_infeasible_count",
                &TrajectoryHandler::getInfeasibleCount
            )
            .def
            (
                "get_sorted_trajectories", 
                [](TrajectoryHandler &self) 
                {
                    self.sort();
                    return nb::make_iterator(nb::type<TrajectoryHandler>(), "trajectory_iterator", self.m_trajectories.begin(), self.m_trajectories.end());
                }
                , nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            ) 
            .def
            (
                "get_cost_functions", 
                [](TrajectoryHandler &self) 
                {
                    return nb::make_iterator<nb::rv_policy::none>(nb::type<TrajectoryHandler>(), "cost_function_iterator", self.m_costFunctions.begin(), self.m_costFunctions.end());
                },
                nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "get_feasability_functions", 
                [](TrajectoryHandler &self) 
                {
                    return nb::make_iterator<nb::rv_policy::none>(nb::type<TrajectoryHandler>(), "feasability_function_iterator", self.m_feasabilityFunctions.begin(), self.m_feasabilityFunctions.end());
                },
                nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "get_other_functions", 
                [](TrajectoryHandler &self) 
                {
                    return nb::make_iterator<nb::rv_policy::none>(nb::type<TrajectoryHandler>(), "other_function_iterator", self.m_otherFunctions.begin(), self.m_otherFunctions.end());
                }, 
                nb::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "reset_Trajectories", 
                &TrajectoryHandler::resetTrajectories, 
                "Resets the trajectories container."
            );
    }

} //plannerCPP

