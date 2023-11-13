#ifndef CHECKCURVATURERATECONSTRAINT_HPP
#define CHECKCURVATURERATECONSTRAINT_HPP

#include "FeasabilityStrategy.hpp"

class TrajectorySample;

/**
 * @class CheckCurvatureRateConstraint
 * @brief A class to check the curvature rate constraints of a trajectory.
 *
 * This class inherits from the FeasabilityStrategy class and is used to check 
 * if a given TrajectorySample meets the curvature rate constraints. The curvature 
 * rate constraints are defined by the wheelbase of the vehicle and the maximum 
 * change in velocity (velocityDeltaMax). The trajectory is evaluated by checking 
 * each of its points. If the rate of change of the curvature (kappaDot) of a sample 
 * exceeds the maximum allowable curvature rate (kappaDotMax), the inFeasability value is 
 * increased by one.
 */
class CheckCurvatureRateConstraint : public FeasabilityStrategy
{
private:
    double m_wheelbase;
    double m_velocityDeltaMax;
public:

    /**
     * @brief Constructor for the CheckCurvatureRateConstraint class.
     *
     * @param wheelbase The wheelbase of the vehicle.
     * @param velocityDeltaMax The maximum change in velocity.
     */
    CheckCurvatureRateConstraint(double wheelbase, double velocityDeltaMax, bool wholeTrajectory);

    /**
     * @brief Evaluate the trajectory and check if it meets the curvature rate constraints.
     *
     * @param trajectory The trajectory to evaluate.
     */
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKCURVATURERATECONSTRAINT_HPP