#ifndef CALCULATECOLLISIONPROBABILITYMAHALANOBIS_HPP
#define CALCULATECOLLISIONPROBABILITYMAHALANOBIS_HPP

#include <map>
#include <string>

#include "CostStrategy.hpp"
#include "geometryMsgs.hpp"

class TrajectorySample;


/**
 * @class CalculateCollisionProbabilityMahalanobis
 * @brief A class to calculate the collision probability for a trajectory using the Mahalanobis distance.
 *
 * This class inherits from the CostStrategy class and is used to calculate 
 * the probability of collision for a given TrajectorySample. The calculation is 
 * based on a provided map of predictions. The Mahalanobis distance, which accounts for 
 * the correlations in the prediction data, is used to compute the collision probability. 
 * The inverse of the Mahalanobis distance is used to compute the collision probability 
 * for each obstacle, and the sum of these probabilities is added to the cost of the trajectory.
 */
class CalculateCollisionProbabilityMahalanobis : public CostStrategy
{
private:
    std::map<int, PredictedObject> m_predictions; /**< A map holding the predicted states of other agents. */
    
public:

    /**
     * @brief Constructor for the CalculateCollisionProbabilityMahalanobis class.
     *
     * This initializes the cost function name to "Prediction", and takes in a map of 
     * predictions as parameter.
     *
     * @param predictions A map holding the predicted states of other agents.
     */
    CalculateCollisionProbabilityMahalanobis(std::string funName, double costWeight, std::map<int, PredictedObject> predictions);

    /**
     * @brief Evaluate the collision probability for a trajectory.
     *
     *
     * @param trajectory The trajectory to evaluate.
     */
    virtual void evaluateTrajectory(TrajectorySample& sample) override;
};


#endif //CALCULATECOLLISIONPROBABILITYMAHALANOBIS_HPP