#ifndef CALCULATEACCELERATIONCOST_HPP
#define CALCULATEACCELERATIONCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

/**
 * @class CalculateAccelerationCost
 * @brief A class to calculate the cost of acceleration for a trajectory.
 *
 * This class inherits from the CostStrategy class and is used to calculate 
 * the cost of acceleration for a given TrajectorySample. The cost is 
 * calculated by squaring the acceleration at each point in the trajectory, 
 * and then integrating these values over the trajectory using Simpson's rule.
 */
class CalculateAccelerationCost : public CostStrategy
{
public:

    /**
     * @brief Constructor for the CalculateAccelerationCost class.
     *
     * This initializes the cost function name to "Acceleration".
     */
    CalculateAccelerationCost(std::string funName, double costWeight);

    /**
     * @brief Evaluate the cost of acceleration for a trajectory.
     *
     * @param trajectory The trajectory to evaluate.
     */
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEACCELERATIONCOST_HPP