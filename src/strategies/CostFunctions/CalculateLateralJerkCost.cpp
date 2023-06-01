#include "CalculateLateralJerkCost.hpp"

CalculateLateralJerkCost::CalculateLateralJerkCost()
    : CostStrategy("Lateral Jerk")
{
}

void CalculateLateralJerkCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    cost = trajectory.m_trajectoryLateral.squaredJerkIntegral(trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost);
}
