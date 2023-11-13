#include "CalculateSteeringRateCost.hpp"

#include <iostream>

#include "TrajectorySample.hpp"

CalculateSteeringRateCost::CalculateSteeringRateCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    std::cout << "Steering Rate Cost not implemented" << std::endl;
}

void CalculateSteeringRateCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
