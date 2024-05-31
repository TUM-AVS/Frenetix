#include "CalculatePositiveOrientationOffsetCost.hpp"

#include <Eigen/Core>
#include <cmath>

#include "TrajectorySample.hpp"

CalculatePositiveOrientationOffsetCost::CalculatePositiveOrientationOffsetCost(std::string funName, double costWeight)
        : CostStrategy(funName, costWeight)
{
}

void CalculatePositiveOrientationOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd theta = trajectory.m_curvilinearSample.theta;

    const Eigen::VectorXd thetaTail = theta.tail(theta.size() / 2).array();

    const Eigen::VectorXd thetaPos = (thetaTail.array() < 0.).select(0., thetaTail);

    double cost = thetaPos.matrix().norm();

    cost += std::pow(thetaPos(thetaPos.size()-1), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}