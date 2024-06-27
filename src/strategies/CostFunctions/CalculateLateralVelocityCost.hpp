#ifndef CALCULATELATERALVELOCITYCOST_HPP
#define CALCULATELATERALVELOCITYCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

/**
 * @class CalculateLateralVelocityCost
 * @brief A class to calculate the cost of lateral velocity for a trajectory.
 *
 * This class inherits from the CostStrategy class and is used to calculate
 * the cost of lateral velocity for a given TrajectorySample. The cost is
 * calculated by squaring the lateral velocity at each point in the trajectory,
 * and then integrating these values over the trajectory using Simpson's rule.
 */
class CalculateLateralVelocityCost : public CostStrategy
{
public:

    /**
     * @brief Constructor for the CalculateLateralVelocityCost class.
     *
     * This initializes the cost function name to "LateralVelocity".
     */
    CalculateLateralVelocityCost(std::string funName, double costWeight);

    /**
     * @brief Evaluate the cost of lateral velocity for a trajectory.
     *
     * @param trajectory The trajectory to evaluate.
     */
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATELATERALVELOCITYCOST_HPP
