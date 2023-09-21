#include "CalculateCollisionProbabilityFast.hpp"

#include <algorithm>
#include <limits>
#include <stdexcept>

CalculateCollisionProbabilityFast::CalculateCollisionProbabilityFast(std::string funName, double costWeight, std::map<int, PredictedObject> predictions, double vehicleLength, double vehicleWidth)
    : CostStrategy(funName, costWeight)
    , m_predictions(predictions)
    , m_vehicleLength(vehicleLength)
    , m_vehicleWidth(vehicleWidth)
{
    std::cerr << "WARNING: This version of CalculateCollisionProbabilityFast is not yet complete! Check results carefully." << std::endl;
}

extern "C" double bvnmvn_(
    const double *lower,
    const double *upper,
    const int32_t *infin,
    const double *correl
);

static inline double bvn_prob(
    const Eigen::AlignedBox<double, 2>& box,
    const Eigen::Matrix<double, 2, 1>& means,
    const Eigen::Matrix<double, 2, 2>& covar
) {
    Eigen::Vector2d stddev = covar.diagonal().array().sqrt();
    
    Eigen::Vector2d norm_lower = (box.min() - means).array() / stddev.array();
    Eigen::Vector2d norm_upper = (box.max() - means).array() / stddev.array();

    Eigen::AlignedBox2d norm_box { norm_lower, norm_upper };

    double rho = covar(0, 1) / std::sqrt(covar(0, 0) * covar(1, 1));

    assert(rho >= -1.0 && rho <= 1.0);

    const int32_t infin[2] = { 2, 2 };
    double result = bvnmvn_(norm_box.min().data(), norm_box.max().data(), &infin[0], &rho);

    return std::abs(result);
}

extern "C" void mvnun_(
    const int32_t *d, const int32_t *n,
    const double *lower, const double *upper,
    const double *means,
    const double *covar,
    const int32_t *maxpts,
    const double *abseps,
    const double *releps,
    double *value,
    int32_t *inform
);

template<int Dim = 2, int Samples = 1>
static inline double mvn_prob(
    const Eigen::AlignedBox<double, Dim>& box,
    const Eigen::Matrix<double, Dim, Samples>& means,
    const Eigen::Matrix<double, Dim, Dim>& covar
) {
    const int32_t dim = Dim;
    const int32_t n = Samples;

#if 1
    // SciPy defaults
    const double releps = 1e-5;
    const double abseps = 1e-5;

    const int32_t maxpts = 1000000 * dim;
#else
    // Relaxed defaults
    const double releps = 1e-3;
    const double abseps = 1e-3;

    const int32_t maxpts = 1000 * Dim;
#endif

    int32_t inform = -1;

    double value = std::numeric_limits<double>::quiet_NaN();

    mvnun_(
        &dim,
        &n,
        box.min().data(),
        box.max().data(),
        means.data(),
        covar.data(),
        &maxpts,
        &abseps,
        &releps,
        &value,
        &inform
    );

    switch (inform) {
    case 0:
        break;
    case 1:
        // Max iterations
        std::cout << "WARNING: Reached max iterations in mvnun" << std::endl;
        break;
    case -1:
        throw std::runtime_error { "Internal error: mvnun did not set inform" };
        break;
    };

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
            if (i >= prediction.predictedPath.size()) { break; }

            Eigen::Vector2d u(trajectory.m_cartesianSample.x[i], trajectory.m_cartesianSample.y[i]);

            const auto& pose = prediction.predictedPath.at(i-1);
            Eigen::Vector2d v = pose.position.head<2>();
            Eigen::Matrix2d cov = pose.covariance.topLeftCorner<2,2>();

            Eigen::AlignedBox2d box { u - offset, u + offset };

            double bvcost = bvn_prob(box, v, cov);
            cost += std::abs(bvcost);
        }
    }

    trajectory.addCostValueToList(m_functionName, cost, cost*m_costWeight);
}

void CalculateCollisionProbabilityFast::printPredictions()
{
    // std::cout << "Predictions: " << std::endl;
}

