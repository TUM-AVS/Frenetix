#ifndef CALCULATELATERALJERKCOST_HPP
#define CALCULATELATERALJERKCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateLateralJerkCost : public CostStrategy
{
public:
    CalculateLateralJerkCost(std::string funName, double costWeight);
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif //CALCULATELATERALJERKCOST_HPP
