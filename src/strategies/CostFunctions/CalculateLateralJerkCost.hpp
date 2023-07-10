#ifndef CALCULATELATERALJERKCOST_HPP
#define CALCULATELATERALJERKCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"

class CalculateLateralJerkCost : public CostStrategy
{
public:
    CalculateLateralJerkCost(std::string funName, double costWeight);
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif //CALCULATELATERALJERKCOST_HPP
