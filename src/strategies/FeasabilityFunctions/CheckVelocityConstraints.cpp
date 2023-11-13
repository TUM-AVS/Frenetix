#include "CheckVelocityConstraints.hpp"

#include <iostream>
#include <string>

class TrajectorySample;

CheckVelocityConstraint::CheckVelocityConstraint(bool wholeTrajectory)
    : FeasabilityStrategy("Velocity Constraint", wholeTrajectory)

{   
    std::cout << m_functionName << ": not implemented yet" << std::endl;
}

void CheckVelocityConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{

}