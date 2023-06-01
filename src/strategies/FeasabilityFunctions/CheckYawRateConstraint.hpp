#ifndef CHECKYAWRATECONSTRAINT_HPP
#define CHECKYAWRATECONSTRAINT_HPP

#include <cmath>

#include "FeasabilityStrategy.hpp"
#include "TrajectorySample.hpp"


class CheckYawRateConstraint : public FeasabilityStrategy
{
private:
    double m_deltaMax;
    double m_wheelbase;
    double m_kappaMax;

public:
    CheckYawRateConstraint(double deltaMax, double wheelbase);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKYAWRATECONSTRAINT_HPP