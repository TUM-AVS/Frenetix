#include "CalculateOrientationOffsetCost.hpp"

#include "TrajectorySample.hpp"


CalculateOrientationOffsetCost::CalculateOrientationOffsetCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateOrientationOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd& theta = trajectory.m_curvilinearSample.theta;

    double cost = (theta.tail(theta.size() / 2).array()).matrix().norm();

    cost += std::pow(theta(theta.size()-1), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
