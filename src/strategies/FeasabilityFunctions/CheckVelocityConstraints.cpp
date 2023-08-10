#include "CheckVelocityConstraints.hpp"

CheckVelocityConstraint::CheckVelocityConstraint(bool wholeTrajectory)
    : FeasabilityStrategy("Velocity Constraint", wholeTrajectory)

{   
    std::cout << m_functionName << ": not implemented yet" << std::endl;
}

void CheckVelocityConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{

}