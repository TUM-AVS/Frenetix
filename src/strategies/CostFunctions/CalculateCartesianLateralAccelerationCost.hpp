#ifndef CALCULATECARTESIANLATERALACCELERATIONCOST_HPP
#define CALCULATECARTESIANLATERALACCELERATIONCOST_HPP

#include <string>
#include "CostStrategy.hpp"

class TrajectorySample;

/**
 * @brief Cost function to penalize high lateral acceleration (cartesian coordinates) a_lat = v^2 * |kappa|.
 *
 * The cost increases quadratically with exceeding a freely selectable reference value.
 */
class CalculateCartesianLateralAccelerationCost : public CostStrategy
{
public:
    /**
     * @param funName Name of the cost function (e.g. "CalculateCartesianLateralAccelerationCost")
     * @param costWeight Weighting of the cost
     * @param latAccRef Reference value of lateral acceleration (m/s^2), below which no cost is incurred
     */
    CalculateCartesianLateralAccelerationCost(std::string funName,
                                              double costWeight,
                                              double latAccRef);

    void evaluateTrajectory(TrajectorySample& trajectory);

private:
    double m_latAccRef;
};

#endif //CALCULATECENTRIPETALACCELERATIONCOST_HPP
