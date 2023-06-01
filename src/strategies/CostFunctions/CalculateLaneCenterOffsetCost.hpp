#ifndef CALCULATELANECENTEROFFSETCOST_HPP
#define CALCULATELANECENTEROFFSETCOST_HPP

#include <iostream>

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateLaneCenterOffsetCost : public CostStrategy
{
public:
    CalculateLaneCenterOffsetCost();
    
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATELANECENTEROFFSETCOST_HPP
