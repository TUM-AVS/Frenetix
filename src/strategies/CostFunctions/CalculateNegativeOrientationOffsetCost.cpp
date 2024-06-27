#include "CalculateNegativeOrientationOffsetCost.hpp"

#include <Eigen/Core>
#include <cmath>

#include "TrajectorySample.hpp"

CalculateNegativeOrientationOffsetCost::CalculateNegativeOrientationOffsetCost(std::string funName, double costWeight)
        : CostStrategy(funName, costWeight)
{
}

void CalculateNegativeOrientationOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    const Eigen::VectorXd theta = trajectory.m_curvilinearSample.theta;

    const Eigen::VectorXd thetaTail = theta.tail(theta.size() / 2).array();

    const Eigen::VectorXd thetaNeg = (thetaTail.array() > 0.).select(0., thetaTail);

    double cost = thetaNeg.matrix().norm();

    cost += std::pow(thetaNeg(thetaNeg.size()-1), 2);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
