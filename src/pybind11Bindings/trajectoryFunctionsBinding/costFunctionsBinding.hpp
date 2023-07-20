#ifndef COSTFUNCTIONBINDING_HPP
#define COSTFUNCTIONBINDING_HPP

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "strategies/CostFunctions/CalculateAccelerationCost.hpp"
#include "strategies/CostFunctions/CalculateDistanceToReferencePathCost.hpp"
#include "strategies/CostFunctions/CalculateJerkCost.hpp"
#include "strategies/CostFunctions/CalculateLaneCenterOffsetCost.hpp"
#include "strategies/CostFunctions/CalculateLateralJerkCost.hpp"
#include "strategies/CostFunctions/CalculateLongitudinalJerkCost.hpp"
#include "strategies/CostFunctions/CalculateLongitudinalVelocityCost.hpp"
#include "strategies/CostFunctions/CalculateOrientationOffsetCost.hpp"
#include "strategies/CostFunctions/CalculateSteeringAngleCost.hpp"
#include "strategies/CostFunctions/CalculateSteeringRateCost.hpp"
#include "strategies/CostFunctions/CalculateVelocityOffsetCost.hpp"
#include "strategies/CostFunctions/CalculateYawCost.hpp"
#include "strategies/CostFunctions/CalculateCollisionProbabilityFast.hpp"
#include "strategies/CostFunctions/CalculateCollisionProbabilityMahalanobis.hpp"
#include "strategies/CostFunctions/CalculateDistanceToObstacleCost.hpp"

namespace py = pybind11;

namespace plannerCPP
{
    void initBindCostFunctions(pybind11::module &m)
    {
        py::class_<CalculateAccelerationCost, CostStrategy, std::shared_ptr<CalculateAccelerationCost>>(m, "CalculateAccelerationCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateAccelerationCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateDistanceToReferencePathCost, CostStrategy, std::shared_ptr<CalculateDistanceToReferencePathCost>>(m, "CalculateDistanceToReferencePathCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateDistanceToReferencePathCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateJerkCost, CostStrategy, std::shared_ptr<CalculateJerkCost>>(m, "CalculateJerkCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateJerkCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateLaneCenterOffsetCost, CostStrategy, std::shared_ptr<CalculateLaneCenterOffsetCost>>(m, "CalculateLaneCenterOffsetCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLaneCenterOffsetCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateLateralJerkCost, CostStrategy, std::shared_ptr<CalculateLateralJerkCost>>(m, "CalculateLateralJerkCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLateralJerkCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateLongitudinalJerkCost, CostStrategy, std::shared_ptr<CalculateLongitudinalJerkCost>>(m, "CalculateLongitudinalJerkCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLongitudinalJerkCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateLongitudinalVelocityCost, CostStrategy, std::shared_ptr<CalculateLongitudinalVelocityCost>>(m, "CalculateLongitudinalVelocityCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLongitudinalVelocityCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateOrientationOffsetCost, CostStrategy, std::shared_ptr<CalculateOrientationOffsetCost>>(m, "CalculateOrientationOffsetCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateOrientationOffsetCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateSteeringAngleCost, CostStrategy, std::shared_ptr<CalculateSteeringAngleCost>>(m, "CalculateSteeringAngleCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateSteeringAngleCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateSteeringRateCost, CostStrategy, std::shared_ptr<CalculateSteeringRateCost>>(m, "CalculateSteeringRateCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateSteeringRateCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateVelocityOffsetCost, CostStrategy, std::shared_ptr<CalculateVelocityOffsetCost>>(m, "CalculateVelocityOffsetCost")
            .def(py::init<std::string, double, double>(), py::arg("function_name"), py::arg("cost_weight"), py::arg("desiredSpeed"))
            .def
            (
                "evaluate_trajectory",
                &CalculateVelocityOffsetCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateYawCost, CostStrategy, std::shared_ptr<CalculateYawCost>>(m, "CalculateYawCost")
            .def(py::init<std::string, double>(), py::arg("function_name"), py::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateYawCost::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateCollisionProbabilityFast, CostStrategy, std::shared_ptr<CalculateCollisionProbabilityFast>>(m, "CalculateCollisionProbabilityFast")
            .def(
                py::init<std::string, double, std::map<int, PredictedObject>, double, double>(),
                py::arg("function_name"),
                py::arg("cost_weight"),
                py::arg("predictions"),
                py::arg("vehicleLength"),
                py::arg("vehicleWidth")
            )
            .def
            (
                "evaluate_trajectory",
                &CalculateCollisionProbabilityFast::evaluateTrajectory,
                py::arg("trajectory")
            )
            .def("printPredictions", &CalculateCollisionProbabilityFast::printPredictions);

        py::class_<CalculateCollisionProbabilityMahalanobis, CostStrategy, std::shared_ptr<CalculateCollisionProbabilityMahalanobis>>(m, "CalculateCollisionProbabilityMahalanobis")
            .def
            (
                py::init<std::string, double, std::map<int, PredictedObject>>(),
                py::arg("function_name"),
                py::arg("cost_weight"),
                py::arg("predictions")
            )
            .def
            (
                "evaluate_trajectory",
                &CalculateCollisionProbabilityMahalanobis::evaluateTrajectory,
                py::arg("trajectory")
            );

        py::class_<CalculateDistanceToObstacleCost, CostStrategy, std::shared_ptr<CalculateDistanceToObstacleCost>>(m, "CalculateDistanceToObstacleCost")
            .def
            (
                py::init<std::string, double, Eigen::Ref<RowMatrixXd>>(),
                py::arg("function_name"),
                py::arg("cost_weight"),
                py::arg("obstacles")
            )
            .def
            (
                "evaluate_trajectory",
                &CalculateDistanceToObstacleCost::evaluateTrajectory,
                py::arg("trajectory")
            );
    }

} //plannerCPP
#endif //COSTFUNCTIONBINDING_HPP