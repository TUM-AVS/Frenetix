#include <spdlog/spdlog.h>

#include "CheckVelocityConstraints.hpp"

class TrajectorySample;

CheckVelocityConstraint::CheckVelocityConstraint(bool wholeTrajectory)
    : FeasabilityStrategy("Velocity Constraint", wholeTrajectory)

{   
    SPDLOG_WARN("CheckVelocityConstraint not implemented yet");
}

void CheckVelocityConstraint::evaluateTrajectory(TrajectorySample& trajectory)
{

}