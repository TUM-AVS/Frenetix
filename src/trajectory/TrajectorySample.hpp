#pragma once

#include <stddef.h>
#include <Eigen/Core>
#include <optional>
#include <string>
#include <utility>
#include <unordered_map>
#include <memory>

#include "CartesianSample.hpp"
#include "CurvilinearSample.hpp"
#include "polynomial.hpp"

class CoordinateSystemWrapper;

struct PlannerState {
    struct Cartesian {
        Eigen::Vector2d pos;
        double orientation;
        double velocity;
        double acceleration;
        double steering_angle;
    } x_0;
    struct Curvilinear {
        Eigen::Vector3d x0_lon;
        Eigen::Vector3d x0_lat;
    } x_cl;
    double wheelbase;
};

PlannerState::Curvilinear computeInitialState(
        std::shared_ptr<CoordinateSystemWrapper> coordinateSystem,
        PlannerState::Cartesian cps,
        double wheelbase, 
        bool lowVelocityMode
);

class TrajectorySample
{
public:
    CartesianSample m_cartesianSample;
    CurviLinearSample m_curvilinearSample;

    using LongitudinalTrajectory = PolynomialTrajectory<4, 3, 2>;
    using FixedLongitudinalTrajectory = PolynomialTrajectory<5, 3, 3>;
    using LateralTrajectory = PolynomialTrajectory<5, 3, 3>;
    static const LongitudinalTrajectory::OrderVectorX0 LongitudinalX0Order;
    static const LongitudinalTrajectory::OrderVectorXD LongitudinalXDOrder;
    std::shared_ptr<LinearTrajectory> m_trajectoryLongitudinal;
    std::shared_ptr<LinearTrajectory> m_trajectoryLateral;

    Eigen::Vector<double, 13> m_samplingParameters;

    std::unordered_map<std::string, std::pair<double,double>> m_costMap;
    std::unordered_map<std::string, double> m_feasabilityMap;

    size_t m_size;
    size_t m_actualSize;
    double m_dT; // = NaN
    double m_cost = 0.0;

    bool m_valid = true;
    bool m_feasible = true;

    // These variables are only used by Python
    std::optional<double> m_harm_occ_module;
    std::optional<double> m_boundaryHarm;

    std::optional<bool> m_collisionDetected;
    std::optional<double> m_egoRisk;
    std::optional<double> m_obstRisk;

    std::optional<int> m_currentTimeStep;
    std::optional<int> m_uniqueId;

    TrajectorySample(double dt,
                     LongitudinalTrajectory trajectoryLongitudinal,
                     LateralTrajectory trajectoryLateral,
                     int uniqueId);

    TrajectorySample(double dt,
                     LongitudinalTrajectory trajectoryLongitudinal,
                     LateralTrajectory trajectoryLateral,
                     int uniqueId,
                     Eigen::VectorXd samplingParameters);

    TrajectorySample(double dt,
                     FixedLongitudinalTrajectory trajectoryLongitudinal,
                     LateralTrajectory trajectoryLateral,
                     int uniqueId);

    TrajectorySample(double dt,
                     FixedLongitudinalTrajectory trajectoryLongitudinal,
                     LateralTrajectory trajectoryLateral,
                     int uniqueId,
                     Eigen::VectorXd samplingParameters);

    static TrajectorySample standstillTrajectory(
        const std::shared_ptr<CoordinateSystemWrapper>& coordinateSystem,
        const PlannerState& state,
        double dt,
        double horizon
    );

    /**
     * @brief Construct a new Trajectory Sample object just with the initial position.
     *
     * @param x0
     * @param y0
     * @param orientation0
     */
    TrajectorySample(double x0,
                     double y0,
                     double orientation0,
                     double acceleration0,
                     double velocity0);

    /**
     * @brief Initialize arrays of the curvilinear and cartesian samples with the specified size.
     *
     * @param size The size to resize the arrays to.
     */
    void initArraysWithSize(size_t size);

    /**
     * @brief Set the current time step for the trajectory sample.
     *
     * @param currentTimeStep The current time step to set.
     */
    void setCurrentTimeStep(int currentTimeStep);

    /**
     * @brief Add a cost value to the list of cost values. The weight must therefore be speicified
     * in the costWeightMap. Add a costWeight to the map if it is not specified yet by
     * handler.addCostWeight(std::string functionName, double weight).
     *
     * @param costFunctionName
     * @param cost
     * @param costWeighted
     */
    void addCostValueToList(std::string costFunctionName, double cost, double costWeighted);

    void addFeasabilityValueToList(std::string costFunctionName, double value);


    size_t size();
};

