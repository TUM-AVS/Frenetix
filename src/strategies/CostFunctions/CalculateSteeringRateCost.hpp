#ifndef CALCULATESTEERINGRATECOST_HPP
#define CALCULATESTEERINGRATECOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateSteeringRateCost : public CostStrategy
{
public:
    CalculateSteeringRateCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATESTEERINGRATECOST_HPP
