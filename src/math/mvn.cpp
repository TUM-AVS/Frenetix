// #undef NDEBUG

#include "mvn.hpp"

#include <assert.h>
#include <Eigen/Geometry>
#include <cmath>

#include "covariance.hpp"


template<int Var1, int Var2, typename Scalar, int Rows, int Cols>
static inline Scalar pearson_correlation(
    const Eigen::Matrix<Scalar, Rows, Cols>& covar,
    const typename Eigen::NumTraits<Scalar>::Real& prec = Eigen::NumTraits<Scalar>::dummy_precision()
) {
    Scalar rho = covar(Var1, Var2) / std::sqrt(covar.diagonal()(Var1) * covar.diagonal()(Var2));

    auto one = static_cast<Scalar>(1.0);

    if (rho + prec < -one || rho - prec > one) {
        throw std::domain_error { "invalid correlation coefficient" };
    }

    return rho;
}

double bvn_prob(
    const Eigen::AlignedBox<double, 2>& box,
    const Eigen::Matrix<double, 2, 1>& means,
    const Eigen::Matrix<double, 2, 2>& covar
) {
    // check_covariance_matrix(covar);

    Eigen::Vector2d stddev = covar.diagonal().array().sqrt();

    Eigen::Vector2d norm_lower = (box.min() - means).array() / stddev.array();
    Eigen::Vector2d norm_upper = (box.max() - means).array() / stddev.array();

    Eigen::AlignedBox2d norm_box { norm_lower, norm_upper };

    double rho = pearson_correlation<0, 1>(covar);

    const int32_t infin[2] = { 2, 2 };
    double result = bvnmvn_(norm_box.min().data(), norm_box.max().data(), &infin[0], &rho);

    assert(!std::isnan(result));

    return std::abs(result);
}

