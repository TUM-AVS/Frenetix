#ifndef CALCULATESTEERINGANGLECOST_HPP
#define CALCULATESTEERINGANGLECOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateSteeringAngleCost : public CostStrategy
{
public:
    CalculateSteeringAngleCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATESTEERINGANGLECOST_HPP
