#ifndef CALCULATEVELOCITYOFFSETCOST_HPP
#define CALCULATEVELOCITYOFFSETCOST_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateVelocityOffsetCost : public CostStrategy
{
private:
    double m_desiredSpeed;
    double m_dT;
    double m_t_min;
    bool m_limitToTmin;
    int m_normOrder;
public:
    CalculateVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed, double dT, double t_min, bool limitToTmin, int normOrder);
    void evaluateTrajectory(TrajectorySample& trajectory);
};

#endif //CALCULATEVELOCITYOFFSETCOST_HPP
