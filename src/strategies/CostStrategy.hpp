#ifndef COSTSTRATEGY_HPP
#define COSTSTRATEGY_HPP

#include "TrajectoryStrategy.hpp"

class CostStrategy: public TrajectoryStrategy
{
public:
    CostStrategy(std::string functionName)
        : TrajectoryStrategy(functionName)
    {
    }
    
    virtual void evaluateTrajectory(TrajectorySample& trajectory) = 0;

};

#endif //COSTSTRATEGY_HPP