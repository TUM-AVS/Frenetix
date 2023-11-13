#ifndef CHECKACCELERATIONCONSTRAINT_HPP
#define CHECKACCELERATIONCONSTRAINT_HPP

#include "FeasabilityStrategy.hpp"

class TrajectorySample;

/**
 * @class CheckAccelerationConstraint
 * @brief A class to check the acceleration constraints of a trajectory.
 *
 * This class inherits from the FeasabilityStrategy class and is used to check 
 * if a given TrajectorySample meets the acceleration constraints. The acceleration 
 * constraints are defined by a maximum acceleration and a switching velocity.
 * The trajectory is evaluated by checking each of its points. For each point,
 * the acceleration is not within the bounds, the inFeasability value is increased by one.
 */
class CheckAccelerationConstraint : public FeasabilityStrategy
{
public:
    /**
     * @brief Constructor for the CheckAccelerationConstraint class.
     *
     * @param switchingVelocity The switching velocity.
     * @param maxAcceleration The maximum acceleration.
     */
    CheckAccelerationConstraint(double switchingVelocity, double maxAcceleration, bool wholeTrajectory);


    /**
     * @brief Evaluate the trajectory and check if it meets the acceleration constraints.
     *
     * @param trajectory The trajectory to evaluate.
     */
    void evaluateTrajectory(TrajectorySample& trajectory);

private:
    double m_switchingVelocity;
    double m_maxAcceleration;
};

#endif //CHECKACCELERATIONCONSTRAINT_HPP
