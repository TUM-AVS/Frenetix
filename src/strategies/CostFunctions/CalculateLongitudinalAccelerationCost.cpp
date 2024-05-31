#include "CalculateLongitudinalAccelerationCost.hpp"

#include <Eigen/Core>

#include "TrajectorySample.hpp"
#include "util.hpp"

CalculateLongitudinalAccelerationCost::CalculateLongitudinalAccelerationCost(std::string funName, double costWeight)
        : CostStrategy(funName, costWeight)
{
}

void CalculateLongitudinalAccelerationCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    Eigen::VectorXd lateralVelSq = trajectory.m_curvilinearSample.sss.array().square();

    double cost = util::simpsonIntegration(lateralVelSq, trajectory.m_dT);

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}