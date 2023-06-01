#include "CalculateYawCost.hpp"

CalculateYawCost::CalculateYawCost()
    : CostStrategy("Yaw Rate")
{
    std::cout << "Yaw Cost not implemented" << std::endl;
}

void CalculateYawCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost);
}
