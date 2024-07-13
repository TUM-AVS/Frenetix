#include <spdlog/spdlog.h>

#include "CalculateYawCost.hpp"

#include "TrajectorySample.hpp"

CalculateYawCost::CalculateYawCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    SPDLOG_WARN("Yaw Cost not implemented");
}

void CalculateYawCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
