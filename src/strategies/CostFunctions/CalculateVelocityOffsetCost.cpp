#include "CalculateVelocityOffsetCost.hpp"

CalculateVelocityOffsetCost::CalculateVelocityOffsetCost(double desiredSpeed)
    : CostStrategy("Velocity Offset")
    , m_desiredSpeed(desiredSpeed)
{
}

void CalculateVelocityOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    const Eigen::VectorXd& vel = trajectory.m_cartesianSample.velocity;
    
    int half_idx = trajectory.size() / 2;
    cost = (vel.segment(half_idx, vel.size() - half_idx - 1) - Eigen::VectorXd::Constant(vel.size() - half_idx - 1, m_desiredSpeed)).cwiseAbs().sum();
    cost += std::pow((vel(vel.size()-1) - m_desiredSpeed), 2);


    trajectory.addCostValueToList(m_functionName, cost);
}
