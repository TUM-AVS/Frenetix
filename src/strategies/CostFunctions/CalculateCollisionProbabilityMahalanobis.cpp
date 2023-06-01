#include "CalculateCollisionProbabilityMahalanobis.hpp"

CalculateCollisionProbabilityMahalanobis::CalculateCollisionProbabilityMahalanobis(std::map<int, std::map<std::string, RowMatrixXd>> predictions)
    : CostStrategy("Prediction")
    , m_predictions(predictions)
{
}

void CalculateCollisionProbabilityMahalanobis::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost = 0.0;
    std::map<int, std::vector<double>> collision_prob_dict;
    for (const auto& [obstacle_id, prediction] : m_predictions) {

        const auto& mean_list = prediction.at("pos_list");
        const auto& cov_list1 = prediction.at("cov_list_0");
        const auto& cov_list2 = prediction.at("cov_list_1");

        std::vector<double> inv_dist;

        for (int i = 1; i < trajectory.m_cartesianSample.x.size(); ++i) 
        {
            if (i < mean_list.size())
            {
                Eigen::Vector2d u(trajectory.m_cartesianSample.x[i], trajectory.m_cartesianSample.y[i]);
                Eigen::Vector2d v = mean_list.row(i - 1);
                Eigen::Matrix2d cov;
                cov << cov_list1.row(i - 1), cov_list2.row(i - 1);
                cov = cov.transpose();
                Eigen::Matrix2d iv = cov.inverse();
                // Calculate Mahalanobis distance manually
                Eigen::Vector2d diff = u - v;
                double mahalanobis = std::sqrt((diff.transpose() * iv * diff)(0, 0));
                inv_dist.push_back(1e-4 / mahalanobis);
            }
            else 
            {
                inv_dist.push_back(0.0);
            }
        }
        collision_prob_dict[obstacle_id] = inv_dist;
        cost += std::accumulate(inv_dist.begin(), inv_dist.end(), 0.0);
    
    }
    trajectory.addCostValueToList(m_functionName, cost);
}