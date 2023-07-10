#include "CalculateLongitudinalJerkCost.hpp"

CalculateLongitudinalJerkCost::CalculateLongitudinalJerkCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateLongitudinalJerkCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    cost = trajectory.m_trajectoryLongitudinal.squaredJerkIntegral(trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
