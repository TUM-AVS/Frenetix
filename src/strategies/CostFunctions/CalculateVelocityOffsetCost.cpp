#include "CalculateVelocityOffsetCost.hpp"

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CalculateVelocityOffsetCost::CalculateVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed, double dT, double t_min, bool limitToTmin, int normOrder)
    : CostStrategy(funName, costWeight)
    , m_desiredSpeed(desiredSpeed)
    , m_dT(dT)
    , m_t_min(t_min)
    , m_limitToTmin(limitToTmin)
    , m_normOrder(normOrder)
{
}

void CalculateVelocityOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    const Eigen::VectorXd vel = trajectory.m_cartesianSample.velocity;

    Eigen::ArrayXd diffs;
    if (m_limitToTmin) {
        auto min_idx = static_cast<int>(m_t_min * m_dT);
        if (min_idx >= vel.size()) {
            throw std::runtime_error { "t_min behind sampling horizon"};
        }

        diffs = vel.head(min_idx).array() - m_desiredSpeed;
    } else {
        diffs = vel.array() - m_desiredSpeed;
    }

    switch (m_normOrder) {
    case 1:
        cost = diffs.matrix().lpNorm<1>();
        break;
    case 2:
        cost = diffs.matrix().lpNorm<2>();
        break;
    default:
        throw std::runtime_error { "invalid norm order" };
    }

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
