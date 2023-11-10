#include "CalculateDistanceToObstacleCost.hpp"

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CalculateDistanceToObstacleCost::CalculateDistanceToObstacleCost(std::string funName, double costWeight, Eigen::Ref<RowMatrixXd> obstacles)
    : CostStrategy(funName, costWeight)
    , m_obstacles(obstacles)
{
}

void CalculateDistanceToObstacleCost::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost {0};

    Eigen::MatrixXd posMatrix(2, trajectory.m_cartesianSample.x.size());
    posMatrix.row(0) = trajectory.m_cartesianSample.x;
    posMatrix.row(1) = trajectory.m_cartesianSample.y;

    for (int i = 0; i < m_obstacles.rows(); ++i) {
        Eigen::Vector2d obstaclePos = m_obstacles.row(i);

        Eigen::MatrixXd diffMatrix = (posMatrix.colwise() - obstaclePos).colwise().squaredNorm();

        const bool use_minimum = true;
        if (use_minimum) {
            cost += 1.0 / diffMatrix.minCoeff();
        } else {
            cost += (1.0 / diffMatrix.array()).sum();
        }
    }

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
