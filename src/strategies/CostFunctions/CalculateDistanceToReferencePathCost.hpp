#ifndef CALCULATEDISTANCETOREFERENCEPATHCOST_HPP
#define CALCULATEDISTANCETOREFERENCEPATHCOST_HPP

#include <iostream>

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateDistanceToReferencePathCost : public CostStrategy
{
public:
    CalculateDistanceToReferencePathCost(std::string funName, double costWeight);
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif //CALCULATEDISTANCETOREFERENCEPATHCOST_HPP
