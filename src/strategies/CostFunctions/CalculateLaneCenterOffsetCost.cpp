#include "CalculateLaneCenterOffsetCost.hpp"

CalculateLaneCenterOffsetCost::CalculateLaneCenterOffsetCost()
    : CostStrategy("Lane Center Offset")
{
    std::cout << "Lane Center Offset Cost not implemented" << std::endl;
}

void CalculateLaneCenterOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define your cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost);
}
