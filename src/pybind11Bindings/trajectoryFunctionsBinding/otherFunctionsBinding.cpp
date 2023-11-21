//pybind includes
#include <pybind11/pybind11.h>
#include <memory>

#include "strategies/Functions/ComputeInitalState.hpp"
#include "strategies/Functions/FillCoordinates.hpp"

#include "CoordinateSystemWrapper.hpp"
#include "otherFunctionsBinding.hpp"
#include "trajectory/TrajectorySample.hpp"

class CoordinateSystemWrapper;
class TrajectoryStrategy;

namespace py = pybind11;

namespace plannerCPP
{
    void initBindOtherFunctions(pybind11::module &m) 
    {
        py::class_<FillCoordinates, TrajectoryStrategy, std::shared_ptr<FillCoordinates>>(m, "FillCoordinates")
            .def(py::init<bool, double, std::shared_ptr<CoordinateSystemWrapper>, double>(),
                py::arg("lowVelocityMode"),
                py::arg("initialOrientation"),
                py::arg("coordinateSystem"),
                py::arg("horizon")
            )
            .def
            (
                "evaluate_trajectory", 
                &FillCoordinates::evaluateTrajectory, 
                py::arg("trajectory")
            );
        
        py::class_<ComputeInitialState, TrajectoryStrategy, std::shared_ptr<ComputeInitialState>>(m, "ComputeInitialState")
            .def(py::init<std::shared_ptr<CoordinateSystemWrapper>, double, double, bool>(),
                py::arg("coordinateSystem"),
                py::arg("wheelBase"),
                py::arg("steeringAngle"),
                py::arg("lowVelocityMode")
            )
            .def
            (
                "evaluate_trajectory", 
                &ComputeInitialState::evaluateTrajectory, 
                py::arg("trajectory")
            );
    }
} //plannerCPP

