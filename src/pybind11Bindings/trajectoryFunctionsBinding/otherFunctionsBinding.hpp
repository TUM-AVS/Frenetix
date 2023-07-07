#ifndef OTHERFUNCTIONBINDING_HPP
#define OTHERFUNCTIONBINDING_HPP

//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "strategies/Functions/FillCoordinates.hpp"
#include "strategies/Functions/ComputeInitalState.hpp"
#include "CoordinateSystemWrapper.hpp"

namespace nb = nanobind;

namespace plannerCPP
{
    void initBindOtherFunctions(nb::module_ &m) 
    {
        nb::class_<FillCoordinates, TrajectoryStrategy>(m, "FillCoordinates")
            .def(nb::init<const Eigen::Ref<Eigen::VectorXd>&, bool, double, std::shared_ptr<CoordinateSystemWrapper>&>(),
                nb::arg("times"), 
                nb::arg("lowVelocityMode"), 
                nb::arg("initialOrientation"), 
                nb::arg("coordinateSystem")
            )
            .def
            (
                "evaluate_trajectory", 
                &FillCoordinates::evaluateTrajectory, 
                nb::arg("trajectory")
            );
        
        nb::class_<ComputeInitialState, TrajectoryStrategy>(m, "ComputeInitialState")
            .def(nb::init<std::shared_ptr<CoordinateSystemWrapper>&, double, double, bool>(),
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

#endif //OTHERFUNCTIONBINDING_HPP
