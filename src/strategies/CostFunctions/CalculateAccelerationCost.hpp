#ifndef CALCULATEACCELERATIONCOST_HPP
#define CALCULATEACCELERATIONCOST_HPP

#include <cmath>

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"


class CalculateAccelerationCost : public CostStrategy
{
private:

public:
    CalculateAccelerationCost();
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEACCELERATIONCOST_HPP