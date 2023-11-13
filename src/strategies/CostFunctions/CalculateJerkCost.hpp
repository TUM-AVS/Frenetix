#ifndef CALCULATEJERKCOST_HPP
#define CALCULATEJERKCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateJerkCost : public CostStrategy
{
public:
    CalculateJerkCost(std::string funName, double costWeight);
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif //CALCULATEJERKCOST_HPP
