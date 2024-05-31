#include "CalculatePositiveVelocityOffsetCost.hpp"

#include <Eigen/Core>
#include <cmath>

#include "TrajectorySample.hpp"

CalculatePositiveVelocityOffsetCost::CalculatePositiveVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed)
        : CostStrategy(funName, costWeight)
        , m_desiredSpeed(desiredSpeed)
{
}

void CalculatePositiveVelocityOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd vel = trajectory.m_cartesianSample.velocity;

    const Eigen::VectorXd offset = vel.tail(vel.size() / 2).array() - m_desiredSpeed;

    const Eigen::VectorXd offsetPos = (offset.array() < 0.).select(0., offset);

    double cost = offsetPos.matrix().norm();

    cost += std::pow(offsetPos(offsetPos.size()-1), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
