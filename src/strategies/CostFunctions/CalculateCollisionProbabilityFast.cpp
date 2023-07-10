#include "CalculateCollisionProbabilityFast.hpp"

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, std::map<std::string, RowMatrixXd>> predictions, double vehicleLength, double vehicleWidth)
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
    for (auto& prediction : m_predictions)
    {
        std::cout << "  " << prediction.first << ": " << std::endl;
        for (auto& prediction_type : prediction.second)
        {
            std::cout << "    " << prediction_type.first << ": " << std::endl;
            std::cout << prediction_type.second << std::endl;
        }
    }
}

