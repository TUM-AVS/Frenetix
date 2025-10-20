#include "CalculateCollisionProbabilityFast.hpp"

#include <cassert>
#include <math/mvn.hpp>
#include <Eigen/Geometry>
#include <cmath>
#include <vector>
#include <algorithm>

#include <math/covariance.hpp>
#include "CartesianSample.hpp"
#include "TrajectorySample.hpp"

#include <spdlog/spdlog.h>

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth, double wheelbaseRear, double prediction_dt, double offCenterWeight)
    : CostStrategy(funName, costWeight)
    , m_predictions(predictions)
    , m_dimensions(vehicleLength, vehicleWidth)
    , m_wheelbaseRear(wheelbaseRear)
    , m_prediction_dt(prediction_dt)
    , m_offCenterWeight(offCenterWeight)
{
}

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth)
    : CalculateCollisionProbabilityFast(funName, costWeight, predictions, vehicleLength, vehicleWidth, vehicleLength / 2.0, 0.1)
{
}

double CalculateCollisionProbabilityFast::integrate(const PoseWithCovariance& pose, const Eigen::Vector2d& egoPos, const Dimensions& egoDimensions, const Dimensions& obsDimensions, const Eigen::Rotation2Dd& egoOrientation)
{
    const Eigen::AlignedBox2d dimbox = egoDimensions.centeredBox();

    if (std::abs(pose.position.z()) >= 1e-9) {
        throw std::runtime_error { "Predicted obstacle position has non-zero Z component, but 3D predictions are not supported" };
    }

    // Predicted obstacle driving direction
    Eigen::Vector2d obsDir = (pose.orientation * Eigen::Vector3d::UnitX()).head<2>().normalized();
    Eigen::Vector2d obsSideDir = obsDir.unitOrthogonal();

    Eigen::Vector2d obsMov = (obsDimensions.length / 2.0) * obsDir;
    Eigen::Vector2d obsSideMov = (obsDimensions.width / 2.0) * obsSideDir;

    Eigen::Vector2d vCenter = pose.position.head<2>();
    Eigen::Vector2d vRear = vCenter - obsMov;
    Eigen::Vector2d vFront = vCenter + obsMov;

    Eigen::Vector2d vRearLeft = vRear - obsSideMov;
    Eigen::Vector2d vRearRight = vRear + obsSideMov;

    Eigen::Vector2d vFrontLeft = vFront - obsSideMov;
    Eigen::Vector2d vFrontRight = vFront + obsSideMov;

    Eigen::Matrix2d egoRot = egoOrientation.toRotationMatrix();
    Eigen::Matrix2d egoInvRot = egoOrientation.inverse().toRotationMatrix();

    // Rotate covariance matrix to account for ego vehicle orientation
    check_covariance_matrix(pose.covariance);
    Eigen::Matrix2d cov = egoInvRot * pose.covariance.topLeftCorner<2,2>() * egoRot;

    auto evalAt = [&] (Eigen::Vector2d obsPos) {
        // Position of ego vehicle relative to obstacle
        Eigen::Vector2d relativePos = egoPos - obsPos;

        // Rotate relative position to cancel ego vehicle orientation
        Eigen::Vector2d axisAlignedPos = egoInvRot * relativePos;

        // Create box around axis aligned ego vehicle position
        Eigen::AlignedBox2d box = dimbox.translated(axisAlignedPos);

        // Note: Means is zero since we already subtracted obsPos above
        return 1e3 * std::abs(bvn_prob(box, Eigen::Vector2d::Zero(), cov));
    };

    const auto probCenter = evalAt(vCenter),
        probRearLeft = evalAt(vRearLeft),
        probRearRight = evalAt(vRearRight),
        probFrontLeft = evalAt(vFrontLeft),
        probFrontRight = evalAt(vFrontRight);

    return probCenter +
        m_offCenterWeight * (probRearLeft + probRearRight + probRearLeft + probFrontLeft + probFrontRight);
}


void CalculateCollisionProbabilityFast::evaluateTrajectory(TrajectorySample& trajectory)
{
    if (!trajectory.m_valid) {
        // Check for unlikely internal logic error
        // (happened too often in the past, causing mysterious bugs...)
        throw std::logic_error { "tried to calculate cost of invalid trajectory" };
    }

    // Check if pediction dt is aligned with trajectory dt
    const double planner_dt = trajectory.m_dT;     
    const double prediction_dt = m_prediction_dt;

    // prediction dt must be >= planner dt
    if (prediction_dt < planner_dt) {
        throw std::runtime_error("Prediction DT is smaller than planner DT. This logic is not supported.");
    }

    const double ratio = prediction_dt / planner_dt; // e.g. 0.5 / 0.25 = 2.0
    
    // Check if ratio is an integer (i.e. prediction dt is a multiple of planner dt)
    const double remainder = std::fmod(ratio, 1.0);
    const double epsilon = 1e-5;

    // If the remainder (e.g. 2.0 % 1.0 = 0.0) is not close to zero, it's not a multiple
    if (std::abs(remainder) > epsilon && std::abs(1.0 - remainder) > epsilon) {
        throw std::runtime_error("Prediction DT is not an integer multiple of planner DT.");
    }

    // Calculate the index step for the evaluation
    const int index_step = static_cast<int>(std::round(ratio));

    // Initialize cost
    double cost = 0.0;

    // SPDLOG_ERROR("Matching results: prediction_dt {}, planner_dt {}, ratio {}, index_step {}", prediction_dt, planner_dt, ratio, index_step);

    const Eigen::AlignedBox2d dimbox = m_dimensions.centeredBox();
    const Eigen::Vector2d wheelbase(m_wheelbaseRear, 0.0);

    // Iterate over all predicted obstacles
    for (const auto& [obstacle_id, prediction] : m_predictions) {
        std::vector<double> inv_dist;

        const Dimensions obsDimensions { prediction.length, prediction.width };

        // Iterate over prediction points
        for (int pred_idx = 1; pred_idx < prediction.predictedPath.size(); ++pred_idx)
        {
            
            // Calculate corresponding planner trajectory index
            const int planner_idx = pred_idx * index_step;

            // SPDLOG_ERROR("matching pred_idx {} to planner_idx {}", pred_idx, planner_idx);
            
            // Check if planner index is within trajectory range
            if (planner_idx >= trajectory.m_cartesianSample.x.size()) { break; }

            // Get ego vehicle position and orientation at the corresponding index
            Eigen::Vector2d u(trajectory.m_cartesianSample.x[planner_idx], trajectory.m_cartesianSample.y[planner_idx]);
            Eigen::Rotation2D egoOrientation(trajectory.m_cartesianSample.theta[planner_idx]);

            // Move rear axle position to center positoin
            u += egoOrientation * wheelbase;

            Eigen::AlignedBox2d box = dimbox.translated(u);

            const auto& pose = prediction.predictedPath.at(pred_idx);
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


            double bvcost = integrate(pose, u, m_dimensions, obsDimensions, egoOrientation);

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

