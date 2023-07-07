#ifndef FEASABILITYFUNCTIONBINDING_HPP
#define FEASABILITYFUNCTIONBINDING_HPP

//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "strategies/FeasabilityFunctions/CheckAccelerationConstraint.hpp"
#include "strategies/FeasabilityFunctions/CheckCurvatureConstraints.hpp"
#include "strategies/FeasabilityFunctions/CheckCurvatureRateConstrains.hpp"
#include "strategies/FeasabilityFunctions/CheckVelocityConstraints.hpp"
#include "strategies/FeasabilityFunctions/CheckYawRateConstraint.hpp"


namespace nb = nanobind;

namespace plannerCPP
{
    void initBindFeasabilityFunctions(nb::module_ &m) 
    {   
        nb::class_<CheckAccelerationConstraint, FeasabilityStrategy>(m, "CheckAccelerationConstraint")
            .def(nb::init<double, double>(),
                nb::arg("switchingVelocity"), nb::arg("maxAcceleration"))
            .def
            (
                "evaluate_trajectory", 
                &CheckAccelerationConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckCurvatureConstraint, FeasabilityStrategy>(m, "CheckCurvatureConstraint")
            .def(nb::init<double, double>(),
                nb::arg("deltaMax"), nb::arg("wheelbase"))
            .def
            (
                "evaluate_trajectory", 
                &CheckCurvatureConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckCurvatureRateConstraint, FeasabilityStrategy>(m, "CheckCurvatureRateConstraint")
            .def(nb::init<double, double>(),
                nb::arg("wheelbase"), nb::arg("velocityDeltaMax"))
            .def
            (
                "evaluate_trajectory", 
                &CheckCurvatureRateConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckVelocityConstraint, FeasabilityStrategy>(m, "CheckVelocityConstraint")
            .def(nb::init<>())
            .def
            (
                "evaluate_trajectory", 
                &CheckVelocityConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );


        nb::class_<CheckYawRateConstraint, FeasabilityStrategy>(m, "CheckYawRateConstraint")
            .def(nb::init<double, double>(),
                nb::arg("deltaMax"), nb::arg("wheelbase"))
            .def
            (
                "evaluate_trajectory", 
                &CheckYawRateConstraint::evaluateTrajectory, 
                nb::arg("trajectory")
            );

    }

} //plannerCPP

#endif //FEASABILITYFUNCTIONBINDING_HPP
