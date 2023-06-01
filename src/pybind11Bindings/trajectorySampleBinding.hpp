#ifndef TRAJECTORYSAMPLEBINDING_HPP
#define TRAJECTORYSAMPLEBINDING_HPP

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "TrajectoryHandler.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindTrajectorySample(pybind11::module &m) 
    {
        py::class_<TrajectorySample>(m, "TrajectorySample")
            .def(py::init<double, double, PolynomialTrajectory<4>&, PolynomialTrajectory<5>&, int>(),
                 py::arg("horizon"), 
                 py::arg("dt"),
                 py::arg("trajectoryLongitudinal"), 
                 py::arg("trajectoryLateral"), 
                 py::arg("uniqueId"))
            .def(py::init<double, double, double, double, double>(),
                 py::arg("x0"), 
                 py::arg("y0"),
                 py::arg("orientation0"), 
                 py::arg("acceleration0"), 
                 py::arg("velocity0"))
            .def_readwrite("m_horizon", &TrajectorySample::m_horizon)
            .def_readwrite("dt", &TrajectorySample::m_dT)
            .def_readwrite("cost", &TrajectorySample::m_cost)
            .def_readwrite("_cost", &TrajectorySample::m_cost)
            .def_readwrite("_coll_detected", &TrajectorySample::m_collisionDetected)
            .def_readwrite("actual_traj_length", &TrajectorySample::m_actualTrajectoryLength)
            .def_readwrite("m_uniqueId", &TrajectorySample::m_uniqueId)
            .def_readwrite("trajectory_long", &TrajectorySample::m_trajectoryLongitudinal)
            .def_readwrite("trajectory_lat", &TrajectorySample::m_trajectoryLateral)
            .def_readwrite("cartesian", &TrajectorySample::m_cartesianSample)
            .def_readwrite("curvilinear", &TrajectorySample::m_curvilinearSample)
            .def_readwrite("boundary_harm", &TrajectorySample::m_boundaryHarm)
            .def_readwrite("_ego_risk", &TrajectorySample::m_egoRisk)
            .def_readwrite("_obst_risk", &TrajectorySample::m_obstRisk)
            .def_readwrite("feasabilityMap", &TrajectorySample::m_feasabilityMap)
            .def_readwrite("costMap", &TrajectorySample::m_costMap)
            .def_readwrite("feasible", &TrajectorySample::m_feasible)
            .def("add_cost_value_to_list", 
                (void (TrajectorySample::*)(std::string, double)) &TrajectorySample::addCostValueToList, 
                py::arg("costFunctionName"), 
                py::arg("cost"),
                "Add a cost value to the list of cost values. The weight must therefore be specified in the costWeightMap."
            )
            .def("add_cost_value_to_list", 
                (void (TrajectorySample::*)(std::string, double, double)) &TrajectorySample::addCostValueToList, 
                py::arg("costFunctionName"), 
                py::arg("cost"), 
                py::arg("weight"),
                "Add a cost value to the list of cost values when the costWeight is not specified in the CostWeightMap."
            );
        
    }


} //plannerCPP


#endif //TRAJECTORYSAMPLEBINDING_HPP
