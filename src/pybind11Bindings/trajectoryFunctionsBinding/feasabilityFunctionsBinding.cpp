//pybind includes
#include <nanobind/nanobind.h>
#include <memory>

#include "strategies/FeasabilityFunctions/CheckAccelerationConstraint.hpp"
#include "strategies/FeasabilityFunctions/CheckCurvatureConstraints.hpp"
#include "strategies/FeasabilityFunctions/CheckCurvatureRateConstrains.hpp"
#include "strategies/FeasabilityFunctions/CheckVelocityConstraints.hpp"
#include "strategies/FeasabilityFunctions/CheckYawRateConstraint.hpp"

#include "feasabilityFunctionsBinding.hpp"
#include "trajectory/TrajectorySample.hpp"

class FeasabilityStrategy;

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindFeasabilityFunctions(nb::module_ &m) 
    {   
        nb::class_<CheckAccelerationConstraint, FeasabilityStrategy>(m, "CheckAccelerationConstraint")
            .def(nb::init<double, double, bool>(),
                nb::arg("switchingVelocity"), nb::arg("maxAcceleration"), nb::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckAccelerationConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckCurvatureConstraint, FeasabilityStrategy>(m, "CheckCurvatureConstraint")
            .def(nb::init<double, double, bool>(),
                nb::arg("deltaMax"), nb::arg("wheelbase"), nb::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckCurvatureConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckCurvatureRateConstraint, FeasabilityStrategy>(m, "CheckCurvatureRateConstraint")
            .def(nb::init<double, double, bool>(),
                nb::arg("wheelbase"), nb::arg("velocityDeltaMax"),  nb::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckCurvatureRateConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckVelocityConstraint, FeasabilityStrategy>(m, "CheckVelocityConstraint")
            .def(nb::init<bool>(),
                nb::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckVelocityConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckYawRateConstraint, FeasabilityStrategy>(m, "CheckYawRateConstraint")
            .def(nb::init<double, double, bool>(),
                nb::arg("deltaMax"), nb::arg("wheelbase"), nb::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckYawRateConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );

    }

} //plannerCPP

