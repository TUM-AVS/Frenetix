#include "CalculateDistanceToObstacleCost.hpp"

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CalculateDistanceToObstacleCost::CalculateDistanceToObstacleCost(std::string funName, double costWeight, Eigen::Ref<RowMatrixXd> obstacles)
    : CostStrategy(funName, costWeight)
    , m_obstacles(obstacles)
{

    if (!isMatrixValid(m_obstacles)) {
        std::cerr << "Warning: Invalid obstacle data (contains NaN or Inf). Obstacle Distance Costs will be set to 0.\n";
        m_obstacles.setZero(); // Optionally reset the matrix to zero
    }

}

void CalculateDistanceToObstacleCost::evaluateTrajectory(TrajectorySample& trajectory)
{

    if (!isVectorValid(trajectory.m_cartesianSample.x) || !isVectorValid(trajectory.m_cartesianSample.y)) {
        std::cerr << "Warning: Invalid trajectory data (contains NaN or Inf). Obstacle Distance Costs will be set to 0.\n";
        trajectory.m_cartesianSample.x.setZero(); // Optionally reset the vectors to zero
        trajectory.m_cartesianSample.y.setZero();
        trajectory.addCostValueToList(m_functionName, 0, 0); // Set cost to 0
        return;
    }

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

bool CalculateDistanceToObstacleCost::isVectorValid(const Eigen::VectorXd& vec) {
    return std::all_of(vec.begin(), vec.end(), [](double v) { return std::isfinite(v); });
}

bool CalculateDistanceToObstacleCost::isMatrixValid(const Eigen::MatrixXd& mat) {
    return std::all_of(mat.data(), mat.data() + mat.size(), [](double v) { return std::isfinite(v); });
}