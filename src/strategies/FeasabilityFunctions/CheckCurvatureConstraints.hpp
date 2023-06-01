#ifndef CHECKCURVATURECONSTRAINT_HPP
#define CHECKCURVATURECONSTRAINT_HPP

#include <cmath>

#include "FeasabilityStrategy.hpp"

class CheckCurvatureConstraint : public FeasabilityStrategy
{
private:
    double m_deltaMax;
    double m_wheelbase;

public:
    CheckCurvatureConstraint(double deltaMax, double wheelbase);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKCURVATURECONSTRAINT_HPP