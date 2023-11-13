#ifndef CALCULATEDISTANCETOREFERENCEPATHCOST_HPP
#define CALCULATEDISTANCETOREFERENCEPATHCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateDistanceToReferencePathCost : public CostStrategy
{
public:
    CalculateDistanceToReferencePathCost(std::string funName, double costWeight);
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif //CALCULATEDISTANCETOREFERENCEPATHCOST_HPP
