#include "CheckCurvatureRateConstrains.hpp"

#include <stddef.h>
#include <Eigen/Core>
#include <cmath>
#include <memory>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CheckCurvatureRateConstraint::CheckCurvatureRateConstraint(double wheelbase, double velocityDeltaMax, bool wholeTrajectory)
    : FeasabilityStrategy("Curvature Rate Constraint", wholeTrajectory)
    , m_wheelbase(wheelbase)
    , m_velocityDeltaMax(velocityDeltaMax)
{   
}

void CheckCurvatureRateConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{
    double inFeasablity {0};

    
    // double steeringAngle {};
    // double kappaDotMax {};
    double kappaDot {};

    size_t lengthToCheck = (m_wholeTrajectory) ? trajectory.m_size : trajectory.m_acutualSize;

    for (size_t iii = 0; iii < lengthToCheck; ++iii) 
    {
        // steeringAngle = std::atan2(m_wheelbase * trajectory.m_cartesianSample.kappa[iii], 1.0);
        // kappaDotMax = m_velocityDeltaMax / (m_wheelbase * std::pow(std::cos(steeringAngle), 2));
        kappaDot = (iii > 0) ? (trajectory.m_cartesianSample.kappa[iii] - trajectory.m_cartesianSample.kappa[iii - 1]) / trajectory.m_dT : 0.0;

        if (std::abs(kappaDot) > 0.4)  inFeasablity++;
    }

    trajectory.addFeasabilityValueToList(m_functionName, inFeasablity);
}