#include "CalculateVelocityOffsetCost.hpp"

CalculateVelocityOffsetCost::CalculateVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed)
    : CostStrategy(funName, costWeight)
    , m_desiredSpeed(desiredSpeed)
{
}

void CalculateVelocityOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    const Eigen::VectorXd& vel = trajectory.m_cartesianSample.velocity;

    int half_idx = vel.size() / 2;
    int half_count = vel.size() - half_idx - 1;
    cost = (vel.segment(half_idx, half_count) - Eigen::VectorXd::Constant(half_count, m_desiredSpeed)).cwiseAbs().sum();
    cost += std::pow((vel(vel.size()-1) - m_desiredSpeed), 2);


    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
