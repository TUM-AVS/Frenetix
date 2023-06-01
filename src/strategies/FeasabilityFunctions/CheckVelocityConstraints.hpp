#ifndef CHECKVELOCITYCONSTRAINT_HPP
#define CHECKVELOCITYCONSTRAINT_HPP

#include "FeasabilityStrategy.hpp"
//not implemented
class CheckVelocityConstraint : public FeasabilityStrategy
{
public:
    CheckVelocityConstraint();
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKVELOCITYCONSTRAINT_HPP