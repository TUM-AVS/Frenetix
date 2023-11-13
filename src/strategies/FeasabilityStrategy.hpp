#ifndef FEASABILITYSTRATEGY_HPP
#define FEASABILITYSTRATEGY_HPP

#include <string>

#include "TrajectoryStrategy.hpp"

class TrajectorySample;

class FeasabilityStrategy: public TrajectoryStrategy
{
public:
    FeasabilityStrategy(std::string functionName, bool wholeTrajectory)
        : TrajectoryStrategy(functionName)
        , m_wholeTrajectory(wholeTrajectory)
    {

    }

    virtual ~FeasabilityStrategy() = default;

    virtual void evaluateTrajectory(TrajectorySample& trajectory) = 0;

protected:
    bool m_wholeTrajectory;
};

#endif //FEASABILITYSTRATEGY_HPP