#include "CalculatePositiveVelocityOffsetCost.hpp"

#include <Eigen/Core>
#include <cmath>
#include <stdexcept>

#include "TrajectorySample.hpp"

CalculatePositiveVelocityOffsetCost::CalculatePositiveVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed, double dT, double t_min, bool limitToTmin, int normOrder)
    : CostStrategy(funName, costWeight)
    , m_desiredSpeed(desiredSpeed)
    , m_dT(dT)
    , m_t_min(t_min)
    , m_limitToTmin(limitToTmin)
    , m_normOrder(normOrder)
{
}

void CalculatePositiveVelocityOffsetCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    const Eigen::VectorXd vel = trajectory.m_cartesianSample.velocity;

    Eigen::ArrayXd diffs;
    if (m_limitToTmin) {
        auto min_idx = static_cast<int>(m_t_min / m_dT);
        if (min_idx >= vel.size()) {
            throw std::runtime_error { "t_min behind sampling horizon"};
        }

        diffs = vel.head(min_idx).array() - m_desiredSpeed;
    } else {
        diffs = vel.array() - m_desiredSpeed;
    }

    // Keep only positive offsets (speeds above desired speed) by setting negative values to 0
    const Eigen::ArrayXd positive_offset = (diffs < 0.).select(0., diffs);

    switch (m_normOrder) {
    case 1:
        cost = positive_offset.matrix().lpNorm<1>();
        break;
    case 2:
        cost = positive_offset.matrix().lpNorm<2>();
        break;
    default:
        throw std::runtime_error { "invalid norm order" };
    }

    // Add extra penalty for final velocity offset (like in original version)
    if (positive_offset.size() > 0) {
        cost += std::pow(positive_offset(positive_offset.size()-1), 2);
    }

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
