#ifndef CHECKACCELERATIONCONSTRAINT_HPP
#define CHECKACCELERATIONCONSTRAINT_HPP

#include "FeasabilityStrategy.hpp"
#include "TrajectorySample.hpp"

class CheckAccelerationConstraint : public FeasabilityStrategy
{
public:
    CheckAccelerationConstraint(double switchingVelocity, double maxAcceleration);
    void evaluateTrajectory(TrajectorySample& trajectory);

private:
    double m_switchingVelocity;
    double m_maxAcceleration;
};

#endif //CHECKACCELERATIONCONSTRAINT_HPP
