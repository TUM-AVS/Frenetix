#ifndef CALCULATENEGATIVEACCELERATIONCOST_HPP
#define CALCULATENEGATIVEACCELERATIONCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateNegativeAccelerationCost : public CostStrategy
{

public:
    CalculateNegativeAccelerationCost(std::string funName, double costWeight);

    void evaluateTrajectory(TrajectorySample& trajectory);

};


#endif //CALCULATENEGATIVEACCELERATIONCOST_HPP
