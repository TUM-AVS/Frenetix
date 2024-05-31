#ifndef CALCULATELONGITUDINALACCELERATIONCOST_HPP
#define CALCULATELONGITUDINALACCELERATIONCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;


class CalculateLongitudinalAccelerationCost : public CostStrategy
{

public:

    CalculateLongitudinalAccelerationCost(std::string funName, double costWeight);

    void evaluateTrajectory(TrajectorySample& trajectory);
};


#endif //CALCULATELONGITUDINALACCELERATIONCOST_HPP
