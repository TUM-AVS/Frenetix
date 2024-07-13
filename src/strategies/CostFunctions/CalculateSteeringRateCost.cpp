#include <spdlog/spdlog.h>

#include "CalculateSteeringRateCost.hpp"

#include "TrajectorySample.hpp"

CalculateSteeringRateCost::CalculateSteeringRateCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    SPDLOG_WARN("Steering Rate Cost not implemented");
}

void CalculateSteeringRateCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
