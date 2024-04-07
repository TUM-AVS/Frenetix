//pybind includes
#include <pybind11/eigen.h> // IWYU pragma: keep
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include <map>
#include <memory>
#include <vector>

#include "TrajectoryHandler.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"

#include "handlerBinding.hpp"

#include "TrajectoryStrategy.hpp"
#include "FeasabilityStrategy.hpp"
#include "CostStrategy.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindHandler(pybind11::module &m) 
    {
        py::class_<TrajectoryHandler>(m, "TrajectoryHandler")
            .def(py::init<double>(), py::arg("dt"))
            .def
            (
                "generate_trajectories", 
                [](TrajectoryHandler &self, const Eigen::Ref<RowMatrixXd> samplingMatrix, bool lowVelocityMode)
                {
                    self.generateTrajectories(samplingMatrix, lowVelocityMode);
                }, 
                py::arg("samplingMatrix"), 
                py::arg("lowVelocityMode")
            )
            .def("sort", &TrajectoryHandler::sort)
            .def
            (
                "add_feasability_function", 
                [](TrajectoryHandler &self, std::shared_ptr<FeasabilityStrategy> function) 
                {
                    self.addFeasabilityFunction(function);
                }, 
                py::arg().noconvert()
            )
            .def
            (
                "add_cost_function", 
                [](TrajectoryHandler &self, std::shared_ptr<CostStrategy> function) 
                {
                    self.addCostFunction(function);
                }, 
                py::arg().noconvert()
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
                py::arg().noconvert()
            )
            .def
            (
                "evaluate_all_current_functions", 
                &TrajectoryHandler::evaluateAllCurrentFunctions, 
                py::arg("calculateAllCosts") = false
            )
            .def
            (
                "evaluate_all_current_functions_concurrent", 
                &TrajectoryHandler::evaluateAllCurrentFunctionsConcurrent, 
                py::arg("calculateAllCosts") = false
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
                    return py::make_iterator(self.m_trajectories.begin(), self.m_trajectories.end());
                }
                , py::keep_alive<0, 1>() // Keep object alive while iterator is used
            ) 
            .def
            (
                "get_cost_functions", 
                [](TrajectoryHandler &self) 
                {
                    return py::make_iterator(self.m_costFunctions.begin(), self.m_costFunctions.end());
                },
                py::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "get_feasability_functions", 
                [](TrajectoryHandler &self) 
                {
                    return py::make_iterator(self.m_feasabilityFunctions.begin(), self.m_feasabilityFunctions.end());
                },
                py::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "get_other_functions", 
                [](TrajectoryHandler &self) 
                {
                return py::make_iterator(self.m_otherFunctions.begin(), self.m_otherFunctions.end());
                }, 
                py::keep_alive<0, 1>() // Keep object alive while iterator is used
            )
            .def
            (
                "reset_Trajectories", 
                &TrajectoryHandler::resetTrajectories, 
                "Resets the trajectories container."
            );
    }

} //plannerCPP

