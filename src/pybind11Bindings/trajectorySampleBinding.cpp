//pybind includes
#include <nanobind/eigen/dense.h> // IWYU pragma: keep
#include <nanobind/nanobind.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/unordered_map.h>
#include <nanobind/stl/optional.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/pair.h>


#include <Eigen/Core>
#include <unordered_map>
#include <optional>
#include <string>
#include <utility>

#include "CartesianSample.hpp"
#include "CurvilinearSample.hpp"
#include "TrajectorySample.hpp"
#include "polynomial.hpp"
#include "CoordinateSystemWrapper.hpp"

#include "trajectorySampleBinding.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindTrajectorySample(nb::module_ &m) 
    {
        nb::class_<PlannerState::Cartesian>(m, "CartesianPlannerState")
            .def(nb::init<Eigen::Vector2d, double, double, double, double>(),
                 nb::arg("pos"),
                 nb::arg("orientation"), 
                 nb::arg("velocity"),
                 nb::arg("acceleration"),
                 nb::arg("steering_angle")
            )
            .def_rw("pos", &PlannerState::Cartesian::pos)
            .def_rw("orientation", &PlannerState::Cartesian::orientation)
            .def_rw("velocity", &PlannerState::Cartesian::velocity)
            .def_rw("acceleration", &PlannerState::Cartesian::acceleration)
            .def_rw("steering_angle", &PlannerState::Cartesian::steering_angle);
        nb::class_<PlannerState::Curvilinear>(m, "CurvilinearPlannerState")
            .def(nb::init<Eigen::Vector3d, Eigen::Vector3d>(),
                 nb::arg("x0_lon"),
                 nb::arg("x0_lat")
            )
            .def_rw("x0_lon", &PlannerState::Curvilinear::x0_lon)
            .def_rw("x0_lat", &PlannerState::Curvilinear::x0_lat);
        nb::class_<PlannerState>(m, "PlannerState")
            .def(nb::init<PlannerState::Cartesian, PlannerState::Curvilinear, double>(),
                 nb::arg("x_0"),
                 nb::arg("x_cl"),
                 nb::arg("wheelbase")
            )
            .def_rw("x_0", &PlannerState::x_0)
            .def_rw("x_cl", &PlannerState::x_cl)
            .def_rw("wheelbase", &PlannerState::wheelbase);

        m.def("compute_initial_state",
                &computeInitialState,
                nb::arg("coordinate_system"),
                nb::arg("x_0"),
                nb::arg("wheelbase"),
                nb::arg("low_velocity_mode")
        );

        nb::class_<TrajectorySample>(m, "TrajectorySample")
            .def(nb::init<double, TrajectorySample::LongitudinalTrajectory, TrajectorySample::LateralTrajectory, int>(),
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
            .def_rw("_cost", &TrajectorySample::m_cost)
            .def_rw("harm_occ_module", &TrajectorySample::m_harm_occ_module)
            .def_rw("_harm_occ_module", &TrajectorySample::m_harm_occ_module)
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
            .def_rw("valid", &TrajectorySample::m_valid)
            .def_prop_rw("sampling_parameters",
                [](TrajectorySample &self) -> Eigen::Ref<Eigen::VectorXd> { return self.m_samplingParameters;},
                [](TrajectorySample &self, const Eigen::Ref<const Eigen::VectorXd> arr) {self.m_samplingParameters = arr;})

            .def_static("compute_standstill_trajectory",
                &TrajectorySample::standstillTrajectory,
                nb::arg("coordinate_system"),
                nb::arg("planner_state"),
                nb::arg("dt"),
                nb::arg("horizon")
            )

            .def("__getstate__",
                [](const TrajectorySample &traj) {
                    nb::dict d;

                    d["dt"]=traj.m_dT;

                    d["cost"]=traj.m_cost;
                    d["feasible"]=traj.m_feasible;
                    d["valid"]=traj.m_valid;

                    d["harm_occ_module"]=traj.m_harm_occ_module;
                    d["boundary_harm"]=traj.m_boundaryHarm;
                    d["ego_risk"]=traj.m_egoRisk;
                    d["obst_risk"]=traj.m_obstRisk;
                    d["coll_detected"]=traj.m_collisionDetected;

                    d["feasabilityMap"]=traj.m_feasabilityMap;
                    d["costMap"]=traj.m_costMap;

                    d["uniqueId"]=traj.m_uniqueId;

                    d["sampling_parameters"]=traj.m_samplingParameters;

                    d["trajectory_long"]=traj.m_trajectoryLongitudinal;
                    d["trajectory_lat"]=traj.m_trajectoryLateral;

                    d["cartesian"]=traj.m_cartesianSample;
                    d["curvilinear"]=traj.m_curvilinearSample;

                    return d;
                })
            .def("__setstate__",
                [](TrajectorySample &traj, nb::dict d) {
                    auto traj_long = nb::cast<TrajectorySample::LongitudinalTrajectory>(d["trajectory_long"]);
                    auto traj_lat = nb::cast<TrajectorySample::LateralTrajectory>(d["trajectory_lat"]);

                    new (&traj) TrajectorySample {
                        nb::cast<double>(d["dt"]),
                        traj_long,
                        traj_lat,
                        nb::cast<int>(d["uniqueId"]),
                        nb::cast<Eigen::VectorXd>(d["sampling_parameters"])
                    };
                    traj.m_cartesianSample = nb::cast<CartesianSample>(d["cartesian"]);
                    traj.m_curvilinearSample = nb::cast<CurviLinearSample>(d["curvilinear"]);

                    traj.m_feasabilityMap = nb::cast<std::unordered_map<std::string, double>>(d["feasabilityMap"]);
                    traj.m_costMap = nb::cast<std::unordered_map<std::string, std::pair<double, double>>>(d["costMap"]);

                    traj.m_cost = nb::cast<double>(d["cost"]);
                    traj.m_feasible = nb::cast<bool>(d["feasible"]);
                    traj.m_valid = nb::cast<bool>(d["valid"]);

                    traj.m_harm_occ_module = nb::cast<std::optional<double>>(d["harm_occ_module"]);
                    traj.m_boundaryHarm = nb::cast<std::optional<double>>(d["boundary_harm"]);
                    traj.m_egoRisk = nb::cast<std::optional<double>>(d["ego_risk"]);
                    traj.m_obstRisk = nb::cast<std::optional<double>>(d["obst_risk"]);
                    traj.m_collisionDetected = nb::cast<std::optional<bool>>(d["coll_detected"]);
                }
            )

            .def("add_cost_value_to_list", 
                (void (TrajectorySample::*)(std::string, double, double)) &TrajectorySample::addCostValueToList, 
                nb::arg("cost_function_name"), 
                nb::arg("cost"),
                nb::arg("weighted_costs"),
                "Add a cost value to the list of cost values. This includes the weighted and unweighted cost."
            );
        
    }


} //plannerCPP

