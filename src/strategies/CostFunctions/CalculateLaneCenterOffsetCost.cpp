#include "CalculateLaneCenterOffsetCost.hpp"

#include <iostream>

#include "TrajectorySample.hpp"

CalculateLaneCenterOffsetCost::CalculateLaneCenterOffsetCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    std::cout << "Lane Center Offset Cost not implemented" << std::endl;
}

void CalculateLaneCenterOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define your cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
