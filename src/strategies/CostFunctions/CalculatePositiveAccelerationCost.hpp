#ifndef CALCULATEPOSITIVEACCELERATIONCOST_HPP
#define CALCULATEPOSITIVEACCELERATIONCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculatePositiveAccelerationCost : public CostStrategy
{

public:
    CalculatePositiveAccelerationCost(std::string funName, double costWeight);

    void evaluateTrajectory(TrajectorySample& trajectory);

};


#endif //CALCULATEPOSITIVEACCELERATIONCOST_HPP
