#include "CalculateVelocityOffsetCost.hpp"

#include <Eigen/Core>
#include <cmath>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CalculateVelocityOffsetCost::CalculateVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed)
    : CostStrategy(funName, costWeight)
    , m_desiredSpeed(desiredSpeed)
{
}

void CalculateVelocityOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    const Eigen::VectorXd& vel = trajectory.m_cartesianSample.velocity;

    cost = (vel.tail(vel.size() / 2).array() - m_desiredSpeed).matrix().norm();

    cost += std::pow((vel(vel.size()-1) - m_desiredSpeed), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
