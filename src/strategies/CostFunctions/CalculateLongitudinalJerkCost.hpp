#ifndef CALCULATELONGITUDINALJERKCOST_HPP
#define CALCULATELONGITUDINALJERKCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateLongitudinalJerkCost : public CostStrategy
{
public:
    CalculateLongitudinalJerkCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATELONGITUDINALJERKCOST_HPP
