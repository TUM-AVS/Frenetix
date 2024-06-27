//pybind includes
#include <nanobind/eigen/dense.h> // IWYU pragma: keep
#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h> // IWYU pragma: keep
#include <nanobind/stl/string.h> // IWYU pragma: keep
#include <nanobind/stl/map.h>

#include <Eigen/Core>
#include <map>
#include <memory>
#include <string>

#include "strategies/CostFunctions/CalculateAccelerationCost.hpp"
#include "strategies/CostFunctions/CalculateCollisionProbabilityFast.hpp"
#include "strategies/CostFunctions/CalculateCollisionProbabilityMahalanobis.hpp"
#include "strategies/CostFunctions/CalculateDistanceToObstacleCost.hpp"
#include "strategies/CostFunctions/CalculateDistanceToReferencePathCost.hpp"
#include "strategies/CostFunctions/CalculateJerkCost.hpp"
#include "strategies/CostFunctions/CalculateLaneCenterOffsetCost.hpp"
#include "strategies/CostFunctions/CalculateLateralAccelerationCost.hpp"
#include "strategies/CostFunctions/CalculateLateralJerkCost.hpp"
#include "strategies/CostFunctions/CalculateLateralVelocityCost.hpp"
#include "strategies/CostFunctions/CalculateLongitudinalAccelerationCost.hpp"
#include "strategies/CostFunctions/CalculateLongitudinalJerkCost.hpp"
#include "strategies/CostFunctions/CalculateLongitudinalVelocityCost.hpp"
#include "strategies/CostFunctions/CalculateNegativeVelocityOffsetCost.hpp"
#include "strategies/CostFunctions/CalculateNegativeOrientationOffsetCost.hpp"
#include "strategies/CostFunctions/CalculateNegativeAccelerationCost.hpp"
#include "strategies/CostFunctions/CalculateOrientationOffsetCost.hpp"
#include "strategies/CostFunctions/CalculatePositiveOrientationOffsetCost.hpp"
#include "strategies/CostFunctions/CalculatePositiveVelocityOffsetCost.hpp"
#include "strategies/CostFunctions/CalculatePositiveAccelerationCost.hpp"
#include "strategies/CostFunctions/CalculateSteeringAngleCost.hpp"
#include "strategies/CostFunctions/CalculateSteeringRateCost.hpp"
#include "strategies/CostFunctions/CalculateVelocityOffsetCost.hpp"
#include "strategies/CostFunctions/CalculateYawCost.hpp"
#include "util.hpp"
#include "trajectory/TrajectorySample.hpp"

#include "costFunctionsBinding.hpp"

class CostStrategy;
struct PredictedObject;

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindCostFunctions(nb::module_ &m)
    {
        nb::class_<CalculateAccelerationCost, CostStrategy>(m, "CalculateAccelerationCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateAccelerationCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateDistanceToReferencePathCost, CostStrategy>(m, "CalculateDistanceToReferencePathCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateDistanceToReferencePathCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateJerkCost, CostStrategy>(m, "CalculateJerkCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateJerkCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateLaneCenterOffsetCost, CostStrategy>(m, "CalculateLaneCenterOffsetCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLaneCenterOffsetCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateLateralJerkCost, CostStrategy>(m, "CalculateLateralJerkCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLateralJerkCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateLateralAccelerationCost, CostStrategy>(m, "CalculateLateralAccelerationCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLateralAccelerationCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateLateralVelocityCost, CostStrategy>(m, "CalculateLateralVelocityCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLateralVelocityCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateLongitudinalJerkCost, CostStrategy>(m, "CalculateLongitudinalJerkCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLongitudinalJerkCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateLongitudinalVelocityCost, CostStrategy>(m, "CalculateLongitudinalVelocityCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLongitudinalVelocityCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateLongitudinalAccelerationCost, CostStrategy>(m, "CalculateLongitudinalAccelerationCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateLongitudinalAccelerationCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateNegativeOrientationOffsetCost, CostStrategy>(m, "CalculateNegativeOrientationOffsetCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateNegativeOrientationOffsetCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateNegativeVelocityOffsetCost, CostStrategy>(m, "CalculateNegativeVelocityOffsetCost")
            .def(nb::init<std::string, double, double>(), nb::arg("function_name"), nb::arg("cost_weight"), nb::arg("desiredSpeed"))
            .def
            (
                "evaluate_trajectory",
                &CalculateNegativeVelocityOffsetCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateOrientationOffsetCost, CostStrategy>(m, "CalculateOrientationOffsetCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateOrientationOffsetCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculatePositiveAccelerationCost, CostStrategy>(m, "CalculatePositiveAccelerationCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculatePositiveAccelerationCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateNegativeAccelerationCost, CostStrategy>(m, "CalculateNegativeAccelerationCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateNegativeAccelerationCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculatePositiveOrientationOffsetCost, CostStrategy>(m, "CalculatePositiveOrientationOffsetCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculatePositiveOrientationOffsetCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculatePositiveVelocityOffsetCost, CostStrategy>(m, "CalculatePositiveVelocityOffsetCost")
            .def(nb::init<std::string, double, double>(), nb::arg("function_name"), nb::arg("cost_weight"), nb::arg("desiredSpeed"))
            .def
            (
                "evaluate_trajectory",
                &CalculatePositiveVelocityOffsetCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateSteeringAngleCost, CostStrategy>(m, "CalculateSteeringAngleCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateSteeringAngleCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateSteeringRateCost, CostStrategy>(m, "CalculateSteeringRateCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateSteeringRateCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateVelocityOffsetCost, CostStrategy>(m, "CalculateVelocityOffsetCost")
            .def(nb::init<std::string, double, double, double, double, bool, int>(), 
                nb::arg("function_name"),
                nb::arg("cost_weight"),
                nb::arg("desiredSpeed"),
                nb::arg("dT"),
                nb::arg("t_min"),
                nb::arg("limit_to_t_min"),
                nb::arg("norm_order")
            )
            .def
            (
                "evaluate_trajectory",
                &CalculateVelocityOffsetCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateYawCost, CostStrategy>(m, "CalculateYawCost")
            .def(nb::init<std::string, double>(), nb::arg("function_name"), nb::arg("cost_weight"))
            .def
            (
                "evaluate_trajectory",
                &CalculateYawCost::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateCollisionProbabilityFast, CostStrategy>(m, "CalculateCollisionProbabilityFast")
            .def(
                nb::init<std::string, double, std::map<int, PredictedObject>, double, double>(),
                nb::arg("function_name"),
                nb::arg("cost_weight"),
                nb::arg("predictions"),
                nb::arg("vehicleLength"),
                nb::arg("vehicleWidth")
            )
            .def(
                nb::init<std::string, double, std::map<int, PredictedObject>, double, double, double, double>(),
                nb::arg("function_name"),
                nb::arg("cost_weight"),
                nb::arg("predictions"),
                nb::arg("vehicle_length"),
                nb::arg("vehicle_width"),
                nb::arg("wheelbase_rear"),
                nb::arg("off_center_weight") = 0.5
            )
            .def
            (
                "evaluate_trajectory",
                &CalculateCollisionProbabilityFast::evaluateTrajectory,
                nb::arg("trajectory")
            )
            .def("printPredictions", &CalculateCollisionProbabilityFast::printPredictions);

        nb::class_<CalculateCollisionProbabilityMahalanobis, CostStrategy>(m, "CalculateCollisionProbabilityMahalanobis")
            .def
            (
                nb::init<std::string, double, std::map<int, PredictedObject>>(),
                nb::arg("function_name"),
                nb::arg("cost_weight"),
                nb::arg("predictions")
            )
            .def
            (
                "evaluate_trajectory",
                &CalculateCollisionProbabilityMahalanobis::evaluateTrajectory,
                nb::arg("trajectory")
            );

        nb::class_<CalculateDistanceToObstacleCost, CostStrategy>(m, "CalculateDistanceToObstacleCost")
            .def
            (
                nb::init<std::string, double, Eigen::Ref<RowMatrixXd>>(),
                nb::arg("function_name"),
                nb::arg("cost_weight"),
                nb::arg("obstacles")
            )
            .def
            (
                "evaluate_trajectory",
                &CalculateDistanceToObstacleCost::evaluateTrajectory,
                nb::arg("trajectory")
            );
    }

} //plannerCPP

