#include "CalculateOrientationOffsetCost.hpp"


CalculateOrientationOffsetCost::CalculateOrientationOffsetCost()
    : CostStrategy("Orientation Offset")
{
}

void CalculateOrientationOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define your cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost);
}
