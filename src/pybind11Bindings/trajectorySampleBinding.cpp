//pybind includes
#include <pybind11/eigen.h> // IWYU pragma: keep
#include <pybind11/pybind11.h>
#include <Eigen/Core>
#include <map>
#include <optional>
#include <string>
#include <utility>

#include "CartesianSample.hpp"
#include "CurvilinearSample.hpp"
#include "TrajectorySample.hpp"
#include "polynomial.hpp"

#include "trajectorySampleBinding.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindTrajectorySample(pybind11::module &m) 
    {
        py::class_<PlannerState::Cartesian>(m, "CartesianPlannerState")
            .def(py::init<Eigen::Vector2d, double, double, double, double>(),
                 py::arg("pos"),
                 py::arg("orientation"), 
                 py::arg("velocity"),
                 py::arg("acceleration"),
                 py::arg("steering_angle")
            )
            .def_readwrite("pos", &PlannerState::Cartesian::pos)
            .def_readwrite("orientation", &PlannerState::Cartesian::orientation)
            .def_readwrite("velocity", &PlannerState::Cartesian::velocity)
            .def_readwrite("acceleration", &PlannerState::Cartesian::acceleration)
            .def_readwrite("steering_angle", &PlannerState::Cartesian::steering_angle);

        py::class_<PlannerState::Curvilinear>(m, "CurvilinearPlannerState")
            .def(py::init<Eigen::Vector3d, Eigen::Vector3d>(),
                 py::arg("x0_lon"),
                 py::arg("x0_lat")
            )
            .def_readwrite("x0_lon", &PlannerState::Curvilinear::x0_lon)
            .def_readwrite("x0_lat", &PlannerState::Curvilinear::x0_lat);

        py::class_<PlannerState>(m, "PlannerState")
            .def(py::init<PlannerState::Cartesian, PlannerState::Curvilinear, double>(),
                 py::arg("x_0"),
                 py::arg("x_cl"),
                 py::arg("wheelbase")
            )
            .def_readwrite("x_0", &PlannerState::x_0)
            .def_readwrite("x_cl", &PlannerState::x_cl)
            .def_readwrite("wheelbase", &PlannerState::wheelbase);

        m.def("compute_initial_state",
                &computeInitialState,
                py::arg("coordinate_system"),
                py::arg("x_0"),
                py::arg("wheelbase"),
                py::arg("low_velocity_mode")
            );

        py::class_<TrajectorySample>(m, "TrajectorySample")
            .def(py::init<double, TrajectorySample::LongitudinalTrajectory, TrajectorySample::LateralTrajectory, int>(),
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
            .def_readwrite("dt", &TrajectorySample::m_dT)
            .def_readwrite("cost", &TrajectorySample::m_cost)
            .def_readwrite("_cost", &TrajectorySample::m_cost)
            .def_readwrite("harm_occ_module", &TrajectorySample::m_harm_occ_module)
            .def_readwrite("_harm_occ_module", &TrajectorySample::m_harm_occ_module)
            .def_readwrite("_coll_detected", &TrajectorySample::m_collisionDetected)
            .def_readwrite("uniqueId", &TrajectorySample::m_uniqueId)
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
            .def_readwrite("valid", &TrajectorySample::m_valid)
            .def_property("sampling_parameters",
                [](TrajectorySample &self) -> Eigen::Ref<Eigen::VectorXd> { return self.m_samplingParameters;},
                [](TrajectorySample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.m_samplingParameters = arr;})

            .def_static("compute_standstill_trajectory",
                &TrajectorySample::standstillTrajectory,
                py::arg("coordinate_system"),
                py::arg("planner_state"),
                py::arg("dt"),
                py::arg("horizon")
            )


            .def(py::pickle(
                [](const TrajectorySample &traj) { // __getstate__
                    using namespace pybind11::literals; // to bring in the `_a` literal
                    py::dict d(
                        "dt"_a=traj.m_dT,

                        "cost"_a=traj.m_cost,
                        "feasible"_a=traj.m_feasible,
                        "valid"_a=traj.m_valid,

                        "harm_occ_module"_a=traj.m_harm_occ_module,
                        "boundary_harm"_a=traj.m_boundaryHarm,
                        "ego_risk"_a=traj.m_egoRisk,
                        "obst_risk"_a=traj.m_obstRisk,
                        "coll_detected"_a=traj.m_collisionDetected,

                        "feasabilityMap"_a=traj.m_feasabilityMap,
                        "costMap"_a=traj.m_costMap,

                        "uniqueId"_a=traj.m_uniqueId,

                        "sampling_parameters"_a=traj.m_samplingParameters,

                        "trajectory_long"_a=traj.m_trajectoryLongitudinal,
                        "trajectory_lat"_a=traj.m_trajectoryLateral,

                        "cartesian"_a=traj.m_cartesianSample,
                        "curvilinear"_a=traj.m_curvilinearSample
                    );

                    return d;
                },
                [](py::dict d) { // __setstate__
                    auto traj_long = d["trajectory_long"].cast<TrajectorySample::LongitudinalTrajectory>();
                    auto traj_lat = d["trajectory_lat"].cast<TrajectorySample::LateralTrajectory>();

                    TrajectorySample traj {
                        d["dt"].cast<double>(),
                        traj_long,
                        traj_lat,
                        d["uniqueId"].cast<int>(),
                        d["sampling_parameters"].cast<Eigen::VectorXd>()
                    };
                    traj.m_cartesianSample = d["cartesian"].cast<CartesianSample>();
                    traj.m_curvilinearSample = d["curvilinear"].cast<CurviLinearSample>();

                    traj.m_feasabilityMap = d["feasabilityMap"].cast<std::map<std::string, double>>();
                    traj.m_costMap = d["costMap"].cast<std::map<std::string, std::pair<double, double>>>();

                    traj.m_cost = d["cost"].cast<double>();
                    traj.m_feasible = d["feasible"].cast<bool>();
                    traj.m_valid = d["valid"].cast<bool>();

                    traj.m_harm_occ_module = d["harm_occ_module"].cast<std::optional<double>>();
                    traj.m_boundaryHarm = d["boundary_harm"].cast<std::optional<double>>();
                    traj.m_egoRisk = d["ego_risk"].cast<std::optional<double>>();
                    traj.m_obstRisk = d["obst_risk"].cast<std::optional<double>>();
                    traj.m_collisionDetected = d["coll_detected"].cast<std::optional<bool>>();

                    return traj;
                }
            ))
            .def("add_cost_value_to_list", 
                (void (TrajectorySample::*)(std::string, double, double)) &TrajectorySample::addCostValueToList, 
                py::arg("cost_function_name"), 
                py::arg("cost"),
                py::arg("weighted_costs"),
                "Add a cost value to the list of cost values. This includes the weighted and unweighted cost."
            );
        
    }


} //plannerCPP

