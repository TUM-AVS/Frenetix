#include "CheckAccelerationConstraint.hpp"

CheckAccelerationConstraint::CheckAccelerationConstraint(double switchingVelocity, double maxAcceleration)
    : FeasabilityStrategy("Acceleration Constraint")
    , m_switchingVelocity(switchingVelocity)
    , m_maxAcceleration(maxAcceleration)
{   
}

void CheckAccelerationConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{
    double inFeasability {0};

    for (size_t iii = 0; iii < trajectory.size(); ++iii) 
    {
        double aMax = (trajectory.m_cartesianSample.velocity[iii] > m_switchingVelocity) ? (m_maxAcceleration * m_switchingVelocity / trajectory.m_cartesianSample.velocity[iii]) : m_maxAcceleration;
        double aMin = -m_maxAcceleration;

        if (!(aMin <= trajectory.m_cartesianSample.velocity[iii] && trajectory.m_cartesianSample.velocity[iii] <= aMax)) 
        {
            inFeasability++;
        }
    }

    trajectory.addFeasabilityValueToList(m_functionName, inFeasability);
}