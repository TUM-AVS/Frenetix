#ifndef CALCULATE_COLLISION_PROBABILITY_FAST_HPP
#define CALCULATE_COLLISION_PROBABILITY_FAST_HPP

#include <Eigen/Core>
#include <map>
#include <string>

#include "CostStrategy.hpp"
#include "geometryMsgs.hpp"

class TrajectorySample;


struct Dimensions {
    double length;
    double width;

    Dimensions(double length_, double width_) : length(length_), width(width_) {
        if (length_ <= 0.0 || width_ <= 0.0) {
            throw std::domain_error { "Dimensions: length and width can't be negative" };
        }
    }

    Eigen::Vector2d corner() const noexcept {
        return Eigen::Vector2d { length / 2.0, width / 2.0 };
    }

    Eigen::AlignedBox2d centeredBox() const noexcept { 
        auto offset = corner();

        return Eigen::AlignedBox2d { -offset, offset };
    }

};

/**
 * @class CalculateCollisionProbabilityFast
 * @brief A class to quickly calculate the collision probability for a trajectory.
 *
 * This class inherits from the CostStrategy class and is used to quickly calculate
 * the probability of collision for a given TrajectorySample. The calculation is
 * based on a provided map of predictions and vehicle dimensions. Currently, the
 * class is not fully implemented.
 */
class CalculateCollisionProbabilityFast : public CostStrategy
{
private:
    std::map<int, PredictedObject> m_predictions; /**< A map holding the predicted states of other agents. */
    double m_vehicleLength; /**< The length of the vehicle. */
    double m_vehicleWidth; /**< The width of the vehicle. */
    double m_wheelbaseRear; /**< The rear wheelbase of the vehicle. */

    /**
     * @brief Calculate the collision probability
     *
     * @param pose Obstacle pose
     * @param ego_pos Ego vehicle position
     * @param offset Ego vehicle extents (length/2, width/2)
     * @param orientation Ego vehicle orientation
     */
    static double integrate(const PoseWithCovariance& pose, const Eigen::Vector2d& ego_pos, const Dimensions& egoDimensions, const Dimensions& obsDimensions, const Eigen::Rotation2Dd& orientation);

public:

    /**
     * @brief Constructor for the CalculateCollisionProbabilityFast class.
     *
     * This initializes the cost function name to "Prediction Cost", and takes in a map of
     * predictions and the dimensions of the vehicle as parameters.
     *
     * @param predictions A map holding the predicted states of other agents.
     * @param vehicleLength The length of the vehicle.
     * @param vehicleWidth The width of the vehicle.
     * @param wheelbaseRear The rear wheelbase of the vehicle.
     */
    CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth, double wheelbaseRear);

    /**
     * @brief Constructor for the CalculateCollisionProbabilityFast class.
     *
     * This initializes the cost function name to "Prediction Cost", and takes in a map of
     * predictions and the dimensions of the vehicle as parameters.
     *
     * @param predictions A map holding the predicted states of other agents.
     * @param vehicleLength The length of the vehicle.
     * @param vehicleWidth The width of the vehicle.
     */
    CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth);

    /**
     * @brief Evaluate the collision probability for a trajectory.
     *
     * This function is not implemented yet.
     *
     * @param trajectory The trajectory to evaluate.
     */
    virtual void evaluateTrajectory(TrajectorySample& trajectory) override;


    /**
     * @brief Print the predictions.
     *
     * This function prints out the predictions stored in the m_predictions map for debugging purposes.
     */
    void printPredictions();
};

#endif // CALCULATE_COLLISION_PROBABILITY_FAST_HPP