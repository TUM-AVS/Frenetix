#include <spdlog/spdlog.h>

#include "CalculateSteeringAngleCost.hpp"

#include "TrajectorySample.hpp"

CalculateSteeringAngleCost::CalculateSteeringAngleCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
    SPDLOG_WARN("Steering Angle Cost not implemented");
}

void CalculateSteeringAngleCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
