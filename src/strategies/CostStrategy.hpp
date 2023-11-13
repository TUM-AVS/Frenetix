#ifndef COSTSTRATEGY_HPP
#define COSTSTRATEGY_HPP

#include <string>

#include "TrajectoryStrategy.hpp"

class TrajectorySample;

class CostStrategy: public TrajectoryStrategy
{
public:
    CostStrategy(std::string functionName, double costWeight)
        : TrajectoryStrategy(functionName)
        , m_costWeight(costWeight)
    {
    }

    virtual ~CostStrategy() = default;

    virtual void evaluateTrajectory(TrajectorySample& trajectory) = 0;

    void updateCostWeight(double costWeight)
    {
        m_costWeight = costWeight;
    }
protected:
    double m_costWeight;
};

#endif //COSTSTRATEGY_HPP