#include "CalculateJerkCost.hpp"

#include <stddef.h>
#include <Eigen/Core>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"

CalculateJerkCost::CalculateJerkCost(std::string funName, double costWeight)
    : CostStrategy(funName, costWeight)
{
}

void CalculateJerkCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    auto size = trajectory.m_cartesianSample.acceleration.size() - 1;

    Eigen::VectorXd jerk = Eigen::VectorXd::Zero(size);

    for (size_t iii = 0; iii < size; ++iii)
    {
        auto diff = trajectory.m_cartesianSample.acceleration(iii + 1) - trajectory.m_cartesianSample.acceleration(iii);
        jerk(iii) = diff / trajectory.m_dT;
    }
    Eigen::VectorXd jerkSq = jerk.array().square();

    cost = util::simpsonIntegration(jerkSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
