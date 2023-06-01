#include "CalculateSteeringAngleCost.hpp"

CalculateSteeringAngleCost::CalculateSteeringAngleCost()
    : CostStrategy("Steering Angle")
{
    std::cout << "Steering Angle Cost not implemented" << std::endl;
}

void CalculateSteeringAngleCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    // Define cost calculation logic here

    trajectory.addCostValueToList(m_functionName, cost);
}
