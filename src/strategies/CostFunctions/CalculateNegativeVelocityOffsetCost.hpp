#ifndef CALCULATENEGATIVEVELOCITYOFFSET_HPP
#define CALCULATENEGATIVEVELOCITYOFFSET_HPP

#include <string>

#include "CostStrategy.hpp"

class TrajectorySample;

class CalculateNegativeVelocityOffsetCost : public CostStrategy
{

private:
    double m_desiredSpeed;

public:
    CalculateNegativeVelocityOffsetCost(std::string funName, double costWeight, double desiredSpeed);

    void evaluateTrajectory(TrajectorySample& trajectory);

};

#endif //CALCULATENEGATIVEVELOCITYOFFSET_HPP
