#include "CalculateCollisionProbabilityFast.hpp"

#include <assert.h>
#include <math/mvn.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>

#include <math/covariance.hpp>
#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth, double wheelbaseRear)
    : CostStrategy(funName, costWeight)
    , m_predictions(predictions)
    , m_vehicleLength(vehicleLength)
    , m_vehicleWidth(vehicleWidth)
    , m_wheelbaseRear(wheelbaseRear)
{
}

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth)
    : CalculateCollisionProbabilityFast(funName, costWeight, predictions, vehicleLength, vehicleWidth, vehicleLength / 2.0)
{
}

double CalculateCollisionProbabilityFast::integrate(const PoseWithCovariance& pose, const Eigen::Vector2d& egoPos, const Dimensions& egoDimensions, const Dimensions& obsDimensions, const Eigen::Rotation2Dd& egoOrientation)
{
    const Eigen::AlignedBox2d dimbox = egoDimensions.centeredBox();

    if (std::abs(pose.position.z()) >= 1e-9) {
        throw std::runtime_error { "Predicted obstacle position has non-zero Z component, but 3D predictions are not supported" };
    }

    // Predicted obstacle driving direction
    Eigen::Vector2d obsDir = (pose.orientation * Eigen::Vector3d::UnitX()).head<2>();
    Eigen::Vector2d obsMov = (obsDimensions.length / 2.0) * obsDir;

    Eigen::Vector2d vCenter = pose.position.head<2>();
    Eigen::Vector2d vRear = vCenter - obsMov;
    Eigen::Vector2d vFront = vCenter + obsMov;

    // Rotate covariance matrix to account for ego vehicle orientation
    Eigen::Matrix2d cov = egoOrientation.inverse() * pose.covariance.topLeftCorner<2,2>() * egoOrientation.toRotationMatrix();

    auto evalAt = [&] (Eigen::Vector2d obsPos) {
        Eigen::Vector2d mpos = egoOrientation.inverse() * (egoPos - obsPos);

        Eigen::AlignedBox2d box = dimbox.translated(mpos);

        return 1e3 * std::abs(bvn_prob(box, Eigen::Vector2d::Zero(), cov));
    };

    const auto probCenter = evalAt(vCenter), probRear = evalAt(vRear), probFront = evalAt(vFront);

    return probCenter + probRear + probFront;
}


void CalculateCollisionProbabilityFast::evaluateTrajectory(TrajectorySample& trajectory)
{
    double cost = 0.0;

    const Dimensions egoDimensions { m_vehicleLength, m_vehicleWidth };

    const Eigen::AlignedBox2d dimbox = egoDimensions.centeredBox();
    const Eigen::Vector2d wheelbase(m_wheelbaseRear, 0.0);

    for (const auto& [obstacle_id, prediction] : m_predictions) {
        std::vector<double> inv_dist;

        const Dimensions obsDimensions { prediction.length, prediction.width };

        for (int i = 1; i < trajectory.m_cartesianSample.x.size(); ++i)
        {
            if (i >= prediction.predictedPath.size()) { break; }

            Eigen::Vector2d u(trajectory.m_cartesianSample.x[i], trajectory.m_cartesianSample.y[i]);

            Eigen::Rotation2D egoOrientation(trajectory.m_cartesianSample.theta[i]);

            // Move rear axle position to center positoin
            u += egoOrientation * wheelbase;

            Eigen::AlignedBox2d box = dimbox.translated(u);

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


            double bvcost = integrate(pose, u, egoDimensions, obsDimensions, egoOrientation);

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

