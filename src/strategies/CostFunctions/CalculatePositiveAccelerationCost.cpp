#include "CalculatePositiveAccelerationCost.hpp"

#include <Eigen/Core>
#include <cmath>

#include "TrajectorySample.hpp"

CalculatePositiveAccelerationCost::CalculatePositiveAccelerationCost(std::string funName, double costWeight)
        : CostStrategy(funName, costWeight)
{
}

void CalculatePositiveAccelerationCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd acc = trajectory.m_cartesianSample.acceleration;

    const Eigen::VectorXd accPos = (acc.array() < 0.).select(0., acc);

    double cost = accPos.matrix().norm();

    cost += std::pow(accPos(accPos.size()-1), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
