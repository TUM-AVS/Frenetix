#include "CalculateDistanceToReferencePathCost.hpp"

CalculateDistanceToReferencePathCost::CalculateDistanceToReferencePathCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateDistanceToReferencePathCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};
    const Eigen::VectorXd& d = trajectory.m_curvilinearSample.d;

    cost = (d.cwiseAbs().sum() + 5 * std::abs(d(d.size() - 1))) / (d.size() + 4);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
