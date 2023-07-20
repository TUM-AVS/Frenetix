#include "CalculateCollisionProbabilityFast.hpp"

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth)
    : CostStrategy(funName, costWeight)
    , m_predictions(predictions)
    , m_vehicleLength(vehicleLength)
    , m_vehicleWidth(vehicleWidth)
{
    std::cout << "CalculateCollisionProbabilityFast not implemented yet" << std::endl;
}

void CalculateCollisionProbabilityFast::evaluateTrajectory(TrajectorySample& trajectory)
{

}

void CalculateCollisionProbabilityFast::printPredictions()
{
    std::cout << "Predictions: " << std::endl;
}

