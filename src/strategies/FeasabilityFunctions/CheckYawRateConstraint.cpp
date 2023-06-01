#include "CheckYawRateConstraint.hpp"

CheckYawRateConstraint::CheckYawRateConstraint(double deltaMax, double wheelbase)
    : FeasabilityStrategy("Yaw rate Constraint")
    , m_deltaMax(deltaMax)
    , m_wheelbase(wheelbase)
{
    m_kappaMax = std::tan(m_deltaMax/m_wheelbase);
}

void CheckYawRateConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{
    double inFeasability {0};

    for (size_t iii = 0; iii < trajectory.size(); ++iii) 
    {
            double yawRate = (iii > 0) ? (trajectory.m_cartesianSample.theta[iii] - trajectory.m_cartesianSample.theta[iii - 1]) / trajectory.m_dT : 0.0;
            double thetaDotMax = m_kappaMax * trajectory.m_cartesianSample.velocity[iii];

            if (std::abs(yawRate) > thetaDotMax) inFeasability++;
    }

    trajectory.addFeasabilityValueToList(m_functionName, inFeasability);
}
