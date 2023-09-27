#ifndef CALCULATE_COLLISION_PROBABILITY_FAST_HPP
#define CALCULATE_COLLISION_PROBABILITY_FAST_HPP

#include "geometryMsgs.hpp"
#include "CostStrategy.hpp"
#include "util.hpp"
#include <map>

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

    /**
     * @brief Calculate the collision probability
     *
     * @param pose Obstacle pose
     * @param ego_pos Ego vehicle position
     * @param offset Ego vehicle extents (length/2, width/2)
     * @param orientation Ego vehicle orientation
     */
    static double integrate(const PoseWithCovariance& pose, const Eigen::Vector2d& ego_pos, const Eigen::Vector2d& offset, double orientation);

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