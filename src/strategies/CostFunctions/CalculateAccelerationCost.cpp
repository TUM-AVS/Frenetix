#include "CalculateAccelerationCost.hpp"

#include <Eigen/Core>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"

CalculateAccelerationCost::CalculateAccelerationCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateAccelerationCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    Eigen::VectorXd accelerationSq = trajectory.m_cartesianSample.acceleration.array().square();

    cost = util::simpsonIntegration(accelerationSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
