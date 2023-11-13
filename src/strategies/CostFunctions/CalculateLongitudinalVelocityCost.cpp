#include "CalculateLongitudinalVelocityCost.hpp"

#include <iostream>

#include "TrajectorySample.hpp"

CalculateLongitudinalVelocityCost::CalculateLongitudinalVelocityCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateLongitudinalVelocityCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    std::cout << m_functionName << ": not implemented" << std::endl;

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
