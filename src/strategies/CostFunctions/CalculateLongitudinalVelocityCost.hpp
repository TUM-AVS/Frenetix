#ifndef CALCULATELONGITUDINALVELOCITYCOST_HPP
#define CALCULATELONGITUDINALVELOCITYCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateLongitudinalVelocityCost : public CostStrategy
{
public:
    CalculateLongitudinalVelocityCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATELONGITUDINALVELOCITYCOST_HPP
