#include "CheckCurvatureConstraints.hpp"

CheckCurvatureConstraint::CheckCurvatureConstraint(double deltaMax, double wheelbase)
    : FeasabilityStrategy("Curvature Constraint")
    , m_deltaMax(deltaMax)
    , m_wheelbase(wheelbase)
{   
}

void CheckCurvatureConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{
    double inFeasablity {0};

    double kappaMax = std::tan(m_deltaMax) / m_wheelbase;

    for (size_t iii = 0; iii < trajectory.size(); ++iii) 
    {
        if (std::abs(trajectory.m_cartesianSample.kappa[iii]) > kappaMax) inFeasablity++;
    }

    trajectory.addFeasabilityValueToList(m_functionName, inFeasablity);
}