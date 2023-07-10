#ifndef CALCULATEYAWCOST_HPP
#define CALCULATEYAWCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateYawCost : public CostStrategy
{
public:
    CalculateYawCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEYAWCOST_HPP
