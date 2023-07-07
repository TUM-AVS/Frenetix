#ifndef FEASABILITYSTRATEGY_HPP
#define FEASABILITYSTRATEGY_HPP

#include "TrajectoryStrategy.hpp"

class FeasabilityStrategy: public TrajectoryStrategy
{
public:
    FeasabilityStrategy(std::string functionName)
        : TrajectoryStrategy(functionName)
    {

    }

    virtual ~FeasabilityStrategy() = default;

    virtual void evaluateTrajectory(TrajectorySample& trajectory) = 0;
};

#endif //FEASABILITYSTRATEGY_HPP