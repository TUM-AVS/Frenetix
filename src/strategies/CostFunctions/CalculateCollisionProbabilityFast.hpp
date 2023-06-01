#ifndef CALCULATE_COLLISION_PROBABILITY_FAST_HPP
#define CALCULATE_COLLISION_PROBABILITY_FAST_HPP

#include "CostStrategy.hpp"
#include "util.hpp"
#include <map>


class CalculateCollisionProbabilityFast : public CostStrategy 
{
private:
    std::map<int, std::map<std::string, RowMatrixXd>> m_predictions;
    double m_vehicleLength;
    double m_vehicleWidth;
public:
    CalculateCollisionProbabilityFast(std::map<int, std::map<std::string, RowMatrixXd>> predictions, double vehicleLength, double vehicleWidth);
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;
    
    void printPredictions();
};

#endif // CALCULATE_COLLISION_PROBABILITY_FAST_HPP