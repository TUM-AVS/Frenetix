#ifndef CALCULATEDISTANCETOOBSTACLECOST_HPP
#define CALCULATEDISTANCETOOBSTACLECOST_HPP

#include <Eigen/Core>
#include <string>

#include "CostStrategy.hpp"
#include "util.hpp"

class TrajectorySample;

/**
 * @class CalculateDistanceToObstacleCost
 * @brief A class to calculate the cost of a trajectory based on the distance to obstacles.
 *
 * This class inherits from the CostStrategy class and is used to calculate the cost of a 
 * TrajectorySample based on its distance to the obstacles. The cost is inversely proportional 
 * to the square of the distance to each obstacle, summed over all obstacles. This encourages 
 * the path planner to select trajectories that keep the vehicle as far away from the obstacles as possible.
 */
class CalculateDistanceToObstacleCost : public CostStrategy
{
private:
    Eigen::Ref<RowMatrixXd> m_obstacles; /**< A matrix holding the positions of the obstacles. */
    bool isVectorValid(const Eigen::VectorXd& vec);
    bool isMatrixValid(const Eigen::MatrixXd& mat);
public:
    CalculateDistanceToObstacleCost(std::string funName, double costWeight, Eigen::Ref<RowMatrixXd> obstacles);
    
    void evaluateTrajectory(TrajectorySample& trajectory) override;
};

#endif // CALCULATEDISTANCETOOBSTACLECOST_HPP