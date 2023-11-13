#ifndef CALCULATEORIENTATIONOFFSETCOST_HPP
#define CALCULATEORIENTATIONOFFSETCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateOrientationOffsetCost : public CostStrategy
{
public:
    CalculateOrientationOffsetCost(std::string funName, double costWeight);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEORIENTATIONOFFSETCOST_HPP
