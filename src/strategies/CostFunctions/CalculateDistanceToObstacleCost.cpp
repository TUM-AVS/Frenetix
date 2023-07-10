#include "CalculateDistanceToObstacleCost.hpp"

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
        Eigen::Vector2d obstaclePos(m_obstacles(i, 0), m_obstacles(i, 1));
        Eigen::MatrixXd diffMatrix = (posMatrix.colwise() - obstaclePos).colwise().norm();
        cost += (1.0 / (diffMatrix.array() * diffMatrix.array())).sum();
    }

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}