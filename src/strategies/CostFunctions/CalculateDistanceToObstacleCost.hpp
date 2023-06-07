#ifndef CALCULATEDISTANCETOOBSTACLECOST_HPP
#define CALCULATEDISTANCETOOBSTACLECOST_HPP

#include "CostStrategy.hpp"
#include "util.hpp"


class CalculateDistanceToObstacleCost : public CostStrategy
{
private:
    Eigen::Ref<RowMatrixXd> m_obstacles;
public:
    CalculateDistanceToObstacleCost(Eigen::Ref<RowMatrixXd> obstacles);
    
    void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif // CALCULATEDISTANCETOOBSTACLECOST_HPP