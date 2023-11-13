#include "CalculateSteeringAngleCost.hpp"

#include <iostream>

#include "TrajectorySample.hpp"

CalculateSteeringAngleCost::CalculateSteeringAngleCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    std::cout << "Steering Angle Cost not implemented" << std::endl;
}

void CalculateSteeringAngleCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
