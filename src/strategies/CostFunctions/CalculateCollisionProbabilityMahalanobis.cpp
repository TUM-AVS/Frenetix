#include "CalculateCollisionProbabilityMahalanobis.hpp"

#include <Eigen/Core>
#include <Eigen/LU>
#include <cmath>
#include <numeric>
#include <vector>

#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CalculateCollisionProbabilityMahalanobis::CalculateCollisionProbabilityMahalanobis(std::string funName, double costWeight, std::map<int, PredictedObject> predictions)
    : CostStrategy(funName, costWeight)
    , m_predictions(predictions)
{
}

void CalculateCollisionProbabilityMahalanobis::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost = 0.0;
    std::map<int, std::vector<double>> collision_prob_dict;
    for (const auto& [obstacle_id, prediction] : m_predictions) {

        std::vector<double> inv_dist;

        for (int i = 1; i < trajectory.m_cartesianSample.x.size(); ++i)
        {
            if (i < prediction.predictedPath.size())
            {
                Eigen::Vector2d u(trajectory.m_cartesianSample.x[i], trajectory.m_cartesianSample.y[i]);
                Eigen::Vector2d v = prediction.predictedPath.at(i-1).position.head<2>();
                Eigen::Matrix2d cov = prediction.predictedPath.at(i-1).covariance.topLeftCorner<2,2>();
                Eigen::Matrix2d iv = cov.inverse();
                // Calculate Mahalanobis distance manually
                Eigen::Vector2d diff = u - v;
                double mahalanobis = std::abs((diff.transpose() * iv * diff)(0, 0));

                // Linearly interpolate the weight based on the prediction timestep
                double weight = 1.0 - static_cast<double>(i) / prediction.predictedPath.size();

                inv_dist.push_back(weight / mahalanobis);
            }
            else
            {
                inv_dist.push_back(0.0);
            }
        }
        collision_prob_dict[obstacle_id] = inv_dist;
        cost += std::accumulate(inv_dist.begin(), inv_dist.end(), 0.0);

    }
    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}
