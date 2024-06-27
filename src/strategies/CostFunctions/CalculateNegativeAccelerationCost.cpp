#include "CalculateNegativeAccelerationCost.hpp"

#include <Eigen/Core>
#include <cmath>

#include "TrajectorySample.hpp"

CalculateNegativeAccelerationCost::CalculateNegativeAccelerationCost(std::string funName, double costWeight)
        : CostStrategy(funName, costWeight)
{
}

void CalculateNegativeAccelerationCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd acc = trajectory.m_cartesianSample.acceleration;

    const Eigen::VectorXd accNeg = (acc.array() > 0.).select(0., acc);

    double cost = accNeg.matrix().norm();

    cost += std::pow(accNeg(accNeg.size()-1), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}

