#include "CalculateOrientationOffsetCost.hpp"

#include "TrajectorySample.hpp"


CalculateOrientationOffsetCost::CalculateOrientationOffsetCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateOrientationOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define your cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
