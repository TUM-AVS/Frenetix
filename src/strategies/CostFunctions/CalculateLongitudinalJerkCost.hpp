#ifndef CALCULATELONGITUDINALJERKCOST_HPP
#define CALCULATELONGITUDINALJERKCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateLongitudinalJerkCost : public CostStrategy
{
public:
    CalculateLongitudinalJerkCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATELONGITUDINALJERKCOST_HPP
