#ifndef CALCULATELONGITUDINALVELOCITYCOST_HPP
#define CALCULATELONGITUDINALVELOCITYCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateLongitudinalVelocityCost : public CostStrategy
{
public:
    CalculateLongitudinalVelocityCost();
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATELONGITUDINALVELOCITYCOST_HPP
