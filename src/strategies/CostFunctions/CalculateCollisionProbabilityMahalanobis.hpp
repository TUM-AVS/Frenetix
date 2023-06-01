#ifndef CALCULATECOLLISIONPROBABILITYMAHALANOBIS_HPP
#define CALCULATECOLLISIONPROBABILITYMAHALANOBIS_HPP

#include <Eigen/Dense>
#include <vector>
#include <map>
#include <numeric>

#include "CostStrategy.hpp"
#include "util.hpp"

class CalculateCollisionProbabilityMahalanobis : public CostStrategy
{
private:
    std::map<int, std::map<std::string, RowMatrixXd>> m_predictions;
public:
    CalculateCollisionProbabilityMahalanobis(std::map<int, std::map<std::string, RowMatrixXd>> predictions);
    virtual void evaluateTrajectory(TrajectorySample& sample) override;
};




#endif //CALCULATECOLLISIONPROBABILITYMAHALANOBIS_HPP