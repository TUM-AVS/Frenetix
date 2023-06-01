#include "CalculateLongitudinalVelocityCost.hpp"

CalculateLongitudinalVelocityCost::CalculateLongitudinalVelocityCost()
    : CostStrategy("Longitudinal Velocity Offset")
{
}

void CalculateLongitudinalVelocityCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    std::cout << m_functionName << ": not implemented" << std::endl;

    trajectory.addCostValueToList(m_functionName, cost);
}
