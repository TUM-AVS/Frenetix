#ifndef CALCULATEVELOCITYOFFSETCOST_HPP
#define CALCULATEVELOCITYOFFSETCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateVelocityOffsetCost : public CostStrategy
{
private:
    double m_desiredSpeed;
public:
    CalculateVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEVELOCITYOFFSETCOST_HPP
