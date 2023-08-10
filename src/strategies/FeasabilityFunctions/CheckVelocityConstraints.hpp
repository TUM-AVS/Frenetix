#ifndef CHECKVELOCITYCONSTRAINT_HPP
#define CHECKVELOCITYCONSTRAINT_HPP

#include "FeasabilityStrategy.hpp"
//not implemented
class CheckVelocityConstraint : public FeasabilityStrategy
{
public:
    CheckVelocityConstraint(bool wholeTrajectory);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKVELOCITYCONSTRAINT_HPP