#include "CalculateCartesianLateralAccelerationCost.hpp"

#include <Eigen/Core>
#include "TrajectorySample.hpp"
#include "CartesianSample.hpp"
#include "util.hpp"

CalculateCartesianLateralAccelerationCost::CalculateCartesianLateralAccelerationCost(
    std::string funName,
    double costWeight,
    double latAccRef)
    : CostStrategy(funName, costWeight)
    , m_latAccRef(latAccRef)
{
}

void CalculateCartesianLateralAccelerationCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    // Load velocity and curvature per sample.
    const Eigen::VectorXd& velocity = trajectory.m_cartesianSample.velocity;
    const Eigen::VectorXd& kappa    = trajectory.m_cartesianSample.kappa;

    // lateral acceleration a_lat = v^2 * |kappa|.
    Eigen::VectorXd latAcc = velocity.array().square() * kappa.array().abs();

    // Deviation from reference value; negative deviations are capped at 0.
    Eigen::VectorXd latAccPos = (latAcc.array() - m_latAccRef).cwiseMax(0.0);

    // Squared deviations.
    Eigen::VectorXd latAccPosSquared = latAcc.array().square();

    // Simpson integration of the squares.
    double cost = util::simpsonIntegration(latAccPosSquared, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost * m_costWeight);
}
