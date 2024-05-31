#ifndef CALCULATEPOSITIVEORIENTATIONOFFSETCOST_HPP
#define CALCULATEPOSITIVEORIENTATIONOFFSETCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculatePositiveOrientationOffsetCost : public CostStrategy
{

public:
    CalculatePositiveOrientationOffsetCost(std::string funName, double costWeight);

    void evaluateTrajectory(TrajectorySample& trajectory);

};

#endif //CALCULATEPOSITIVEORIENTATIONOFFSETCOST_HPP