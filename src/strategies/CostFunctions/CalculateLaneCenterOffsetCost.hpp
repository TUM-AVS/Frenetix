#ifndef CALCULATELANECENTEROFFSETCOST_HPP
#define CALCULATELANECENTEROFFSETCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateLaneCenterOffsetCost : public CostStrategy
{
public:
    CalculateLaneCenterOffsetCost(std::string funName, double costWeight);
    
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATELANECENTEROFFSETCOST_HPP
