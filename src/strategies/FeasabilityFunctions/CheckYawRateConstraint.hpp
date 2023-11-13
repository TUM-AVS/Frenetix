#ifndef CHECKYAWRATECONSTRAINT_HPP
#define CHECKYAWRATECONSTRAINT_HPP

#include "FeasabilityStrategy.hpp"

class TrajectorySample;

/**
 * @class CheckYawRateConstraint
 * @brief A class to check the yaw rate constraints of a trajectory.
 *
 * This class inherits from the FeasabilityStrategy class and is used to check 
 * if a given TrajectorySample meets the yaw rate constraints. The yaw rate 
 * constraints are defined by the deltaMax (maximum steering angle) and the wheelbase 
 * of the vehicle. The trajectory is evaluated by checking each of its points. 
 * If the rate of change of the yaw (yawRate) of a sample exceeds the maximum 
 * allowable yaw rate (thetaDotMax), the inFeasability value is increased by one.
 */
class CheckYawRateConstraint : public FeasabilityStrategy
{
private:
    double m_deltaMax;
    double m_wheelbase;
    double m_kappaMax;

public:

    /**
     * @brief Constructor for the CheckYawRateConstraint class.
     *
     * @param deltaMax The maximum steering angle.
     * @param wheelbase The wheelbase of the vehicle.
     * @param wholeTrajectory Whether to check the whole trajectory including the enlarged part.
     */
    CheckYawRateConstraint(double deltaMax, double wheelbase, bool wholeTrajectory);

    /**
     * @brief Evaluate the trajectory and check if it meets the yaw rate constraints.
     *
     * @param trajectory The trajectory to evaluate.
     */
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKYAWRATECONSTRAINT_HPP