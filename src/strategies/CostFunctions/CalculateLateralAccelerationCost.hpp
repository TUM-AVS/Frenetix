#ifndef CALCULATELATERALACCELERATIONCOST_HPP
#define CALCULATELATERALACCELERATIONCOST_HPP


#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;


class CalculateLateralAccelerationCost : public CostStrategy
{

public:

    CalculateLateralAccelerationCost(std::string funName, double costWeight);

    void evaluateTrajectory(TrajectorySample& trajectory);
};



#endif //CALCULATELATERALACCELERATIONCOST_HPP
