#ifndef CALCULATESTEERINGRATECOST_HPP
#define CALCULATESTEERINGRATECOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateSteeringRateCost : public CostStrategy
{
public:
    CalculateSteeringRateCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATESTEERINGRATECOST_HPP
