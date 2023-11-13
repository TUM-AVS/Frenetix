#include "CheckAccelerationConstraint.hpp"

#include <stddef.h>
#include <Eigen/Core>
#include <memory>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CheckAccelerationConstraint::CheckAccelerationConstraint(double switchingVelocity, double maxAcceleration, bool wholeTrajectory)
    : FeasabilityStrategy("Acceleration Constraint", wholeTrajectory)
    , m_switchingVelocity(switchingVelocity)
    , m_maxAcceleration(maxAcceleration)
{   
}

void CheckAccelerationConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{
    double inFeasability {0};

    size_t lengthToCheck = (m_wholeTrajectory) ? trajectory.m_size : trajectory.m_acutualSize;

    for (size_t iii = 0; iii < lengthToCheck; ++iii) 
    {
        double aMax = (trajectory.m_cartesianSample.velocity[iii] > m_switchingVelocity) ? (m_maxAcceleration * m_switchingVelocity / trajectory.m_cartesianSample.velocity[iii]) : m_maxAcceleration;
        double aMin = -m_maxAcceleration;

        if (!(aMin <= trajectory.m_cartesianSample.acceleration[iii] && trajectory.m_cartesianSample.acceleration[iii] <= aMax)) 
        {
            inFeasability++;
        }
    }

    trajectory.addFeasabilityValueToList(m_functionName, inFeasability);
}