#include "CalculateCollisionProbabilityFast.hpp"

#include <algorithm>

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth)
    : CostStrategy(funName, costWeight)
    , m_predictions(predictions)
    , m_vehicleLength(vehicleLength)
    , m_vehicleWidth(vehicleWidth)
{
    std::cerr << "WARNING: This version of CalculateCollisionProbabilityFast is not yet complete! Check results carefully." << std::endl;
}

extern "C" void mvnun_(
    const int32_t *d, const int32_t *n,
    double *lower, double *upper, double *means, double *covar,
    const int32_t *maxpts,
    const double *abseps,
    const double *releps,
    double *value,
    const int32_t *inform
);

static inline double mvn_prob(
    Eigen::Vector2d lower,
    Eigen::Vector2d upper,
    Eigen::Vector2d means,
    Eigen::Matrix2d covar
) {
    const int d = 2;
    const int n = 1;

    const double releps = 1e-6;
    const double abseps = 1e-2;

    const int maxpts = 100;

    const int inform = 0;

    double value;

    mvnun_(
        &d,
        &n,
        lower.data(),
        upper.data(),
        means.data(),
        covar.data(),
        &maxpts,
        &abseps,
        &releps,
        &value,
        &inform
    );

    return value;
}

void CalculateCollisionProbabilityFast::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost = 0.0;

    Eigen::Vector2d offset(m_vehicleLength / 2.0, m_vehicleWidth / 2.0);

    for (const auto& [obstacle_id, prediction] : m_predictions) {

        std::vector<double> inv_dist;

        for (int i = 1; i < trajectory.m_cartesianSample.x.size(); ++i)
        {
            if (i >= prediction.predictedPath.size()) { continue; }

            Eigen::Vector2d u(trajectory.m_cartesianSample.x[i], trajectory.m_cartesianSample.y[i]);
            const auto& pose = prediction.predictedPath.at(i-1);
            Eigen::Vector2d v = pose.position.head<2>();
            Eigen::Matrix2d cov = pose.covariance.topLeftCorner<2,2>();

            Eigen::Vector2d lower = v - offset;
            Eigen::Vector2d upper = v + offset;

            cost += mvn_prob(lower, upper, u, cov);
        }
    }

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}

void CalculateCollisionProbabilityFast::printPredictions()
{
    // std::cout << "Predictions: " << std::endl;
}

