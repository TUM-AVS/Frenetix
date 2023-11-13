#ifndef CHECKCURVATURECONSTRAINT_HPP
#define CHECKCURVATURECONSTRAINT_HPP

#include "FeasabilityStrategy.hpp"

class TrajectorySample;

/**
 * @class CheckCurvatureConstraint
 * @brief A class to check the curvature constraints of a trajectory.
 *
 * This class inherits from the FeasabilityStrategy class and is used to check 
 * if a given TrajectorySample meets the curvature constraints. The curvature 
 * constraints are defined by a maximum steering angle (deltaMax) and the wheelbase 
 * of the vehicle. The trajectory is evaluated by checking each of its points. 
 * If the curvature of a point exceeds the maximum allowable curvature (kappaMax), 
 * the inFeasability value is increased by one.
 */
class CheckCurvatureConstraint : public FeasabilityStrategy
{
private:
    double m_deltaMax;
    double m_wheelbase;

public:

    /**
     * @brief Constructor for the CheckCurvatureConstraint class.
     *
     * @param deltaMax The maximum steering angle.
     * @param wheelbase The wheelbase of the vehicle.
     */
    CheckCurvatureConstraint(double deltaMax, double wheelbase, bool wholeTrajectory);

    /**
     * @brief Evaluate the trajectory and check if it meets the curvature constraints.
     *
     * @param trajectory The trajectory to evaluate.
     */
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CHECKCURVATURECONSTRAINT_HPP