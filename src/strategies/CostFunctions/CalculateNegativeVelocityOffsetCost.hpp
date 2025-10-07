#ifndef CALCULATENEGATIVEVELOCITYOFFSET_HPP
#define CALCULATENEGATIVEVELOCITYOFFSET_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateNegativeVelocityOffsetCost : public CostStrategy
{

private:
    double m_desiredSpeed;
    double m_dT;
    double m_t_min;
    bool m_limitToTmin;
    int m_normOrder;

public:
    CalculateNegativeVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed, double dT, double t_min, bool limitToTmin, int normOrder);

    void evaluateTrajectory(TrajectorySample& trajectory);

};

#endif //CALCULATENEGATIVEVELOCITYOFFSET_HPP
