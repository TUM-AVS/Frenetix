#include "CalculateYawCost.hpp"

#include <iostream>

#include "TrajectorySample.hpp"

CalculateYawCost::CalculateYawCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    std::cout << "Yaw Cost not implemented" << std::endl;
}

void CalculateYawCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
