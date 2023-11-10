#include "CalculateLongitudinalJerkCost.hpp"

#include <cmath>

#include "TrajectorySample.hpp"
#include "polynomial.hpp"

CalculateLongitudinalJerkCost::CalculateLongitudinalJerkCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateLongitudinalJerkCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    cost = std::sqrt(trajectory.m_trajectoryLongitudinal.squaredJerkIntegral(trajectory.m_dT));

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
