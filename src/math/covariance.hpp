#pragma once

#include <Eigen/Dense>
#include <Eigen/Cholesky>

#include <spdlog/spdlog.h>

#include <stdexcept>
#include <iostream>

template<typename Scalar, int Rows, int Cols>
bool is_symmetric(
    const Eigen::Matrix<Scalar, Rows, Cols>& mat
) {
    static_assert(Rows == Cols, "matrix needs to be square");

    const Scalar symmetric_epsilon = 1e-12;
    return mat.isApprox(mat.transpose(), symmetric_epsilon);
}

template<typename Scalar, int Rows, int Cols>
bool is_positive_semidefinite(
    const Eigen::Matrix<Scalar, Rows, Cols>& mat
) {
    static_assert(Rows == Cols, "matrix needs to be square");

    Eigen::LDLT<Eigen::Matrix<Scalar, Rows, Cols>> ldlt(mat);
    return ldlt.info() == Eigen::Success && ldlt.isPositive();
}

class invalid_covariance_matrix_error : public std::domain_error {
protected:
    explicit invalid_covariance_matrix_error(const std::string& message)
        : std::domain_error(message) {}
};

class not_symmetric_error : public invalid_covariance_matrix_error {
public:
    not_symmetric_error()
        : invalid_covariance_matrix_error("Covariance Matrix is not symmetric") {}
};

class not_positive_semidefinite_error : public invalid_covariance_matrix_error {
public:
    not_positive_semidefinite_error()
        : invalid_covariance_matrix_error("Covariance Matrix is not positive semi-definite") {}
};

template<typename Scalar, int Rows, int Cols>
void check_covariance_matrix(
    const Eigen::Matrix<Scalar, Rows, Cols>& covariance
) {
    if (!is_symmetric(covariance)) {
        SPDLOG_WARN("covar not symmetric!");
        throw not_symmetric_error {};
    }
    if (!is_positive_semidefinite(covariance)) {
        SPDLOG_WARN("covar not positive semi-definite!");
        throw not_positive_semidefinite_error {};
    }
}

