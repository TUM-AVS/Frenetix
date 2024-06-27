#include "CalculateLateralAccelerationCost.hpp"

#include <Eigen/Core>

#include "TrajectorySample.hpp"
#include "util.hpp"

CalculateLateralAccelerationCost::CalculateLateralAccelerationCost(std::string funName, double costWeight)
        : CostStrategy(funName, costWeight)
{
}

void CalculateLateralAccelerationCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    Eigen::VectorXd lateralVelSq = trajectory.m_curvilinearSample.ddd.array().square();

    double cost = util::simpsonIntegration(lateralVelSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}