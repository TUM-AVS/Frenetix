#include "CalculateDistanceToReferencePathCost.hpp"

#include <Eigen/Core>

#include "CurvilinearSample.hpp"
#include "TrajectorySample.hpp"

CalculateDistanceToReferencePathCost::CalculateDistanceToReferencePathCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateDistanceToReferencePathCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    const Eigen::VectorXd& d = trajectory.m_curvilinearSample.d;

    // cost = (d.cwiseAbs().sum() + 5 * std::abs(d(d.size() - 1))) / (d.size() + 4);
    cost = d.tail(d.size() / 2).norm();

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
