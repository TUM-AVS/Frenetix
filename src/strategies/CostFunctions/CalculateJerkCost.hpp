#ifndef CALCULATEJERKCOST_HPP
#define CALCULATEJERKCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"
#include "util.hpp"

class CalculateJerkCost : public CostStrategy
{
public:
    CalculateJerkCost();
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif //CALCULATEJERKCOST_HPP
