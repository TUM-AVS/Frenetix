#include "CalculateLongitudinalJerkCost.hpp"

CalculateLongitudinalJerkCost::CalculateLongitudinalJerkCost()
    : CostStrategy("Longitudinal Jerk")
{
}

void CalculateLongitudinalJerkCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    cost = trajectory.m_trajectoryLongitudinal.squaredJerkIntegral(trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost);
}
