#ifndef CHECKCURVATURERATECONSTRAINT_HPP
#define CHECKCURVATURERATECONSTRAINT_HPP

#include <cmath>

#include "FeasabilityStrategy.hpp"

class CheckCurvatureRateConstraint : public FeasabilityStrategy
{
private:
    double m_wheelbase;
    double m_velocityDeltaMax;
public:
    CheckCurvatureRateConstraint(double wheelbase, double velocityDeltaMax);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKCURVATURERATECONSTRAINT_HPP