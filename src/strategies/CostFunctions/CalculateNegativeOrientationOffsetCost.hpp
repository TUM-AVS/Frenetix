#ifndef CALCULATENEGATIVEORIENTATIONOFFSETCOST_HPP
#define CALCULATENEGATIVEORIENTATIONOFFSETCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateNegativeOrientationOffsetCost : public CostStrategy
{

public:
    CalculateNegativeOrientationOffsetCost(std::string funName, double costWeight);

    void evaluateTrajectory(TrajectorySample& trajectory);

};

#endif //CALCULATENEGATIVEORIENTATIONOFFSETCOST_HPP
