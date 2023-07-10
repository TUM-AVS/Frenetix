#ifndef TRAJECTORYSAMPLEBINDING_HPP
#define TRAJECTORYSAMPLEBINDING_HPP

//pybind includes
#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

#include "TrajectoryHandler.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindTrajectorySample(nb::module_ &m)
    {
        nb::class_<TrajectorySample>(m, "TrajectorySample")
            .def(nb::init<double, PolynomialTrajectory<4>&, PolynomialTrajectory<5>&, int>(),
                 nb::arg("dt"),
                 nb::arg("trajectoryLongitudinal"),
                 nb::arg("trajectoryLateral"),
                 nb::arg("uniqueId"))
            .def(nb::init<double, double, double, double, double>(),
                 nb::arg("x0"),
                 nb::arg("y0"),
                 nb::arg("orientation0"),
                 nb::arg("acceleration0"),
                 nb::arg("velocity0"))
            .def_rw("dt", &TrajectorySample::m_dT)
            .def_rw("cost", &TrajectorySample::m_cost)
            .def_rw("_coll_detected", &TrajectorySample::m_collisionDetected)
            .def_rw("uniqueId", &TrajectorySample::m_uniqueId)
            .def_rw("trajectory_long", &TrajectorySample::m_trajectoryLongitudinal)
            .def_rw("trajectory_lat", &TrajectorySample::m_trajectoryLateral)
            .def_rw("cartesian", &TrajectorySample::m_cartesianSample)
            .def_rw("curvilinear", &TrajectorySample::m_curvilinearSample)
            .def_rw("boundary_harm", &TrajectorySample::m_boundaryHarm)
            .def_rw("_ego_risk", &TrajectorySample::m_egoRisk)
            .def_rw("_obst_risk", &TrajectorySample::m_obstRisk)
            .def_rw("feasabilityMap", &TrajectorySample::m_feasabilityMap)
            .def_rw("costMap", &TrajectorySample::m_costMap)
            .def_rw("feasible", &TrajectorySample::m_feasible)
            .def_prop_rw("sampling_parameters",
                [](TrajectorySample &self) -> Eigen::VectorXd { return self.m_samplingParameters;},
                [](TrajectorySample &self, const nb::DRef<const Eigen::VectorXd>& arr) {self.m_samplingParameters = arr;})
            .def("add_cost_value_to_list",
                (void (TrajectorySample::*)(std::string, double, double)) &TrajectorySample::addCostValueToList,
                nb::arg("cost_function_name"),
                nb::arg("cost"),
                nb::arg("weighted_costs"),
                "Add a cost value to the list of cost values. This includes the weighted and unweighted cost."
            );
    }


} //plannerCPP


#endif //TRAJECTORYSAMPLEBINDING_HPP
