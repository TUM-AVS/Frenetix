#include "CheckCurvatureConstraints.hpp"

CheckCurvatureConstraint::CheckCurvatureConstraint(double deltaMax, double wheelbase, bool wholeTrajectory)
    : FeasabilityStrategy("Curvature Constraint", wholeTrajectory)
    , m_deltaMax(deltaMax)
    , m_wheelbase(wheelbase)
{   
}

void CheckCurvatureConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{
    double inFeasablity {0};

    double kappaMax = std::tan(m_deltaMax) / m_wheelbase;

    size_t lengthToCheck = (m_wholeTrajectory) ? trajectory.m_size : trajectory.m_acutualSize;

    for (size_t iii = 2; iii < lengthToCheck-2; ++iii) 
    {
        if (std::abs(trajectory.m_cartesianSample.kappa[iii]) > kappaMax) inFeasablity++;
    }

    trajectory.addFeasabilityValueToList(m_functionName, inFeasablity);
}