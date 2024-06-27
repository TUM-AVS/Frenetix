#include "CalculateNegativeVelocityOffsetCost.hpp"

#include <Eigen/Core>
#include <cmath>

#include "TrajectorySample.hpp"

CalculateNegativeVelocityOffsetCost::CalculateNegativeVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed)
        : CostStrategy(funName, costWeight)
        , m_desiredSpeed(desiredSpeed)
{
}

void CalculateNegativeVelocityOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd vel = trajectory.m_cartesianSample.velocity;

    const Eigen::VectorXd offset = vel.tail(vel.size() / 2).array() - m_desiredSpeed;

    const Eigen::VectorXd offsetNeg = (offset.array() > 0.).select(0., offset);

    double cost = offset.matrix().norm();

    cost += std::pow(offsetNeg(offsetNeg.size()-1), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
