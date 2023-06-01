#ifndef CALCULATEYAWCOST_HPP
#define CALCULATEYAWCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateYawCost : public CostStrategy
{
public:
    CalculateYawCost();
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEYAWCOST_HPP
