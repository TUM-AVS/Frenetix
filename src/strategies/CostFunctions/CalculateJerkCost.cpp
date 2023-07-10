#include "CalculateJerkCost.hpp"

CalculateJerkCost::CalculateJerkCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateJerkCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    Eigen::VectorXd jerk = Eigen::VectorXd::Zero(trajectory.m_cartesianSample.acceleration.size() - 1);

    for (size_t iii = 0; iii < trajectory.size() - 1; ++iii) 
    {
        jerk(iii) = trajectory.m_cartesianSample.acceleration(iii + 1);
    }
    Eigen::VectorXd jerkSq = jerk.array().square();

    cost = util::simpsonIntegration(jerkSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
