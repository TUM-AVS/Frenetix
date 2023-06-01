#ifndef CALCULATEVELOCITYOFFSETCOST_HPP
#define CALCULATEVELOCITYOFFSETCOST_HPP

#include "CostStrategy.hpp"
#include "TrajectorySample.hpp"

class CalculateVelocityOffsetCost : public CostStrategy
{
private:
    double m_desiredSpeed;
public:
    CalculateVelocityOffsetCost(double desiredSpeed);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEVELOCITYOFFSETCOST_HPP
