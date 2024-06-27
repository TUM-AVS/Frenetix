#include "CalculateLateralVelocityCost.hpp"

#include <Eigen/Core>

#include "TrajectorySample.hpp"
#include "util.hpp"

CalculateLateralVelocityCost::CalculateLateralVelocityCost(std::string funName, double costWeight)
        : CostStrategy(funName, costWeight)
{
}

void CalculateLateralVelocityCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    Eigen::VectorXd lateralVelSq = trajectory.m_curvilinearSample.dd.array().square();

    double cost = util::simpsonIntegration(lateralVelSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
