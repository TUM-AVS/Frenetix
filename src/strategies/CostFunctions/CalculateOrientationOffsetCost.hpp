#ifndef CALCULATEORIENTATIONOFFSETCOST_HPP
#define CALCULATEORIENTATIONOFFSETCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateOrientationOffsetCost : public CostStrategy
{
public:
    CalculateOrientationOffsetCost();
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEORIENTATIONOFFSETCOST_HPP
