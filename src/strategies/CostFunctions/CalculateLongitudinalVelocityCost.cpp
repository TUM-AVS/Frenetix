#include "CalculateLongitudinalVelocityCost.hpp"

#include "TrajectorySample.hpp"

#include <util.hpp>

CalculateLongitudinalVelocityCost::CalculateLongitudinalVelocityCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateLongitudinalVelocityCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    Eigen::VectorXd longitudinalVelSq = trajectory.m_curvilinearSample.ss.array().square();

    double cost = util::simpsonIntegration(longitudinalVelSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
