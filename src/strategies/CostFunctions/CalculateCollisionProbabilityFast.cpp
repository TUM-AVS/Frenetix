#include "CalculateCollisionProbabilityFast.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

#include <math/mvn.hpp>

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth)
    : CostStrategy(funName, costWeight)
    , m_predictions(predictions)
    , m_vehicleLength(vehicleLength)
    , m_vehicleWidth(vehicleWidth)
{
}

double CalculateCollisionProbabilityFast::integrate(const PoseWithCovariance& pose, const Eigen::Vector2d& pos, const Eigen::Vector2d& offset, double orientation)
{
    Eigen::Vector2d v = pose.position.head<2>();

    Eigen::Rotation2D ego_rot(orientation);

    Eigen::Vector2d mpos = ego_rot.inverse() * (pos - v);

    Eigen::AlignedBox2d box { mpos - offset, mpos + offset };

    // Rotate covariance matrix to account for ego vehicle orientation
    Eigen::Matrix2d cov = ego_rot.inverse() * pose.covariance.topLeftCorner<2,2>() * ego_rot.toRotationMatrix();

    return std::abs(bvn_prob(box, Eigen::Vector2d::Zero(), cov));
}

void CalculateCollisionProbabilityFast::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost = 0.0;

    Eigen::Vector2d offset(m_vehicleLength / 2.0, m_vehicleWidth / 2.0);

    for (const auto& [obstacle_id, prediction] : m_predictions) {

        std::vector<double> inv_dist;

        for (int i = 1; i < trajectory.m_cartesianSample.x.size(); ++i)
        {
            if (i >= prediction.predictedPath.size()) { break; }

            Eigen::Vector2d u(trajectory.m_cartesianSample.x[i], trajectory.m_cartesianSample.y[i]);
            Eigen::AlignedBox2d box { u - offset, u + offset };

            const auto& pose = prediction.predictedPath.at(i-1);
            Eigen::Vector2d v = pose.position.head<2>();

            // Check if the distance between the vehicles is larger than ~7 meters
            // If true, skip calculating the probability since it will be very low
            //
            // NOTE: Adapted from Python code, but with a large threshold to be safe
            // since the compared points aren't exactly the same
            // (exterior distance vs center distance, 3 box vs 1 box)
            if (box.squaredExteriorDistance(v) > 50.0) {
                continue;
            }

            double bvcost = integrate(pose, u, offset, trajectory.m_cartesianSample.theta[i]);

            cost += bvcost;
            assert(!std::isnan(cost));
        }
    }

    assert(!std::isnan(cost));

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}

void CalculateCollisionProbabilityFast::printPredictions()
{
    // std::cout << "Predictions: " << std::endl;
}

