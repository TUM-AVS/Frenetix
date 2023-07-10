#ifndef CALCULATESTEERINGANGLECOST_HPP
#define CALCULATESTEERINGANGLECOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateSteeringAngleCost : public CostStrategy
{
public:
    CalculateSteeringAngleCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATESTEERINGANGLECOST_HPP
