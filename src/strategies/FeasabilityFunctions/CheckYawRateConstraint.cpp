#include "CheckYawRateConstraint.hpp"

#include <stddef.h>
#include <Eigen/Core>
#include <cmath>
#include <memory>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CheckYawRateConstraint::CheckYawRateConstraint(double deltaMax, double wheelbase, bool wholeTrajectory)
    : FeasabilityStrategy("Yaw rate Constraint", wholeTrajectory)
    , m_deltaMax(deltaMax)
    , m_wheelbase(wheelbase)
{
    m_kappaMax = std::tan(m_deltaMax) / m_wheelbase;
}

void CheckYawRateConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{
    double inFeasability {0};

    size_t lengthToCheck = (m_wholeTrajectory) ? trajectory.m_size : trajectory.m_acutualSize;

    for (size_t iii = 0; iii < lengthToCheck; ++iii) 
    {
            double yawRate = (iii > 0) ? (trajectory.m_cartesianSample.theta[iii] - trajectory.m_cartesianSample.theta[iii - 1]) / trajectory.m_dT : 0.0;
            double thetaDotMax = m_kappaMax * trajectory.m_cartesianSample.velocity[iii];

            if (std::abs(std::round(yawRate * 100000) / 100000.0) > thetaDotMax) inFeasability++; 
    }

    trajectory.addFeasabilityValueToList(m_functionName, inFeasability);
}
