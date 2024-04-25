//pybind includes
#include <nanobind/nanobind.h>
#include <memory>
#include <nanobind/stl/shared_ptr.h>

#include "strategies/Functions/ComputeInitalState.hpp"
#include "strategies/Functions/FillCoordinates.hpp"

#include "CoordinateSystemWrapper.hpp"
#include "otherFunctionsBinding.hpp"
#include "trajectory/TrajectorySample.hpp"

class CoordinateSystemWrapper;
class TrajectoryStrategy;

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindOtherFunctions(nb::module_ &m) 
    {
        nb::class_<FillCoordinates, TrajectoryStrategy>(m, "FillCoordinates")
            .def(nb::init<bool, double, std::shared_ptr<CoordinateSystemWrapper>, double>(),
                nb::arg("lowVelocityMode"),
                nb::arg("initialOrientation"),
                nb::arg("coordinateSystem"),
                nb::arg("horizon")
            )
            .def
            (
                "evaluate_trajectory", 
                &FillCoordinates::evaluateTrajectory, 
                nb::arg("trajectory")
            );
        
        nb::class_<ComputeInitialState, TrajectoryStrategy>(m, "ComputeInitialState")
            .def(nb::init<std::shared_ptr<CoordinateSystemWrapper>, double, double, bool>(),
                nb::arg("coordinateSystem"),
                nb::arg("wheelBase"),
                nb::arg("steeringAngle"),
                nb::arg("lowVelocityMode")
            )
            .def
            (
                "evaluate_trajectory", 
                &ComputeInitialState::evaluateTrajectory, 
                nb::arg("trajectory")
            );
    }
} //plannerCPP

