#include "CalculateLateralJerkCost.hpp"

CalculateLateralJerkCost::CalculateLateralJerkCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateLateralJerkCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    cost = trajectory.m_trajectoryLateral.squaredJerkIntegral(trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
