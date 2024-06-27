#ifndef CALCULATEPOSITIVEVELOCITYOFFSETCOST_HPP
#define CALCULATEPOSITIVEVELOCITYOFFSETCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculatePositiveVelocityOffsetCost : public CostStrategy
{

private:
    double m_desiredSpeed;

public:
    CalculatePositiveVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed);

    void evaluateTrajectory(TrajectorySample& trajectory);

};

#endif //CALCULATEPOSITIVEVELOCITYOFFSETCOST_HPP
