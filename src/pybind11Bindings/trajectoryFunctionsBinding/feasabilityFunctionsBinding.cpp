//pybind includes
#include <pybind11/pybind11.h>
#include <memory>

#include "strategies/FeasabilityFunctions/CheckAccelerationConstraint.hpp"
#include "strategies/FeasabilityFunctions/CheckCurvatureConstraints.hpp"
#include "strategies/FeasabilityFunctions/CheckCurvatureRateConstrains.hpp"
#include "strategies/FeasabilityFunctions/CheckVelocityConstraints.hpp"
#include "strategies/FeasabilityFunctions/CheckYawRateConstraint.hpp"

#include "feasabilityFunctionsBinding.hpp"
#include "trajectory/TrajectorySample.hpp"

class FeasabilityStrategy;

namespace py = pybind11;

namespace plannerCPP
{
    void initBindFeasabilityFunctions(pybind11::module &m) 
    {   
        py::class_<CheckAccelerationConstraint, FeasabilityStrategy, std::shared_ptr<CheckAccelerationConstraint>>(m, "CheckAccelerationConstraint")
            .def(py::init<double, double, bool>(),
                py::arg("switchingVelocity"), py::arg("maxAcceleration"), py::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckAccelerationConstraint::evaluateTrajectory, 
                py::arg("trajectory")
            );


        py::class_<CheckCurvatureConstraint, FeasabilityStrategy, std::shared_ptr<CheckCurvatureConstraint>>(m, "CheckCurvatureConstraint")
            .def(py::init<double, double, bool>(),
                py::arg("deltaMax"), py::arg("wheelbase"), py::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckCurvatureConstraint::evaluateTrajectory, 
                py::arg("trajectory")
            );


        py::class_<CheckCurvatureRateConstraint, FeasabilityStrategy, std::shared_ptr<CheckCurvatureRateConstraint>>(m, "CheckCurvatureRateConstraint")
            .def(py::init<double, double, bool>(),
                py::arg("wheelbase"), py::arg("velocityDeltaMax"),  py::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckCurvatureRateConstraint::evaluateTrajectory, 
                py::arg("trajectory")
            );


        py::class_<CheckVelocityConstraint, FeasabilityStrategy, std::shared_ptr<CheckVelocityConstraint>>(m, "CheckVelocityConstraint")
            .def(py::init<bool>(),
                py::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckVelocityConstraint::evaluateTrajectory, 
                py::arg("trajectory")
            );


        py::class_<CheckYawRateConstraint, FeasabilityStrategy, std::shared_ptr<CheckYawRateConstraint>>(m, "CheckYawRateConstraint")
            .def(py::init<double, double, bool>(),
                py::arg("deltaMax"), py::arg("wheelbase"), py::arg("wholeTrajectory"))
            .def
            (
                "evaluate_trajectory", 
                &CheckYawRateConstraint::evaluateTrajectory, 
                py::arg("trajectory")
            );

    }

} //plannerCPP

