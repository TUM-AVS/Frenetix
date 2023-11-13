#ifndef TRAJECTORYSTRATEGY_HPP
#define TRAJECTORYSTRATEGY_HPP

#include <string>

class TrajectorySample;

class TrajectoryStrategy
{
public:
    TrajectoryStrategy(std::string functionName)
        : m_functionName(functionName)
    {

    }

    virtual ~TrajectoryStrategy() = default;

    virtual void evaluateTrajectory(TrajectorySample& trajectory) = 0;
    std::string getFunctionName() const{ return m_functionName; }

protected:
    const std::string m_functionName;
};

#endif //TRAJECTORYSTRATEGY_HPP