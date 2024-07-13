#include <spdlog/spdlog.h>

#include "CalculateLaneCenterOffsetCost.hpp"

#include "TrajectorySample.hpp"

CalculateLaneCenterOffsetCost::CalculateLaneCenterOffsetCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    SPDLOG_WARN("Lane Center Offset Cost not implemented");
}

void CalculateLaneCenterOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define your cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
