#include "CalculateAccelerationCost.hpp"

CalculateAccelerationCost::CalculateAccelerationCost()
    : CostStrategy("Acceleration")
{
}

void CalculateAccelerationCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    Eigen::VectorXd accelerationSq = trajectory.m_cartesianSample.acceleration.array().square();

    cost = util::simpsonIntegration(accelerationSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost);
}
