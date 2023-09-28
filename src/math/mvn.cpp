// #undef NDEBUG

#include "mvn.hpp"
#include "covariance.hpp"

#include <algorithm>
#include <limits>

double bvn_prob(
    const Eigen::AlignedBox<double, 2>& box,
    const Eigen::Matrix<double, 2, 1>& means,
    const Eigen::Matrix<double, 2, 2>& covar
) {
    check_covariance_matrix(covar);

    Eigen::Vector2d stddev = covar.diagonal().array().sqrt();

    Eigen::Vector2d norm_lower = (box.min() - means).array() / stddev.array();
    Eigen::Vector2d norm_upper = (box.max() - means).array() / stddev.array();

    Eigen::AlignedBox2d norm_box { norm_lower, norm_upper };

    double rho = covar(0, 1) / std::sqrt(covar(0, 0) * covar(1, 1));

    assert(!std::isnan(rho));
    if (rho < -1.0 || rho > 1.0) {
        throw std::domain_error { "invalid correlation coefficient" };
    }
    assert(rho >= -1.0 && rho <= 1.0);

    const int32_t infin[2] = { 2, 2 };
    double result = bvnmvn_(norm_box.min().data(), norm_box.max().data(), &infin[0], &rho);

    assert(!std::isnan(result));

    return std::abs(result);
}

