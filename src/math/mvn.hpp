#pragma once

#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <spdlog/spdlog.h>

#include "mvndst.hpp"

double bvn_prob(
    const Eigen::AlignedBox<double, 2>& box,
    const Eigen::Matrix<double, 2, 1>& means,
    const Eigen::Matrix<double, 2, 2>& covar
);

template<int Dim = 2, int Samples = 1>
double mvn_prob(
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
        SPDLOG_WARN("Reached max iterations in mvnun");
        break;
    case -1:
        throw std::runtime_error { "Internal error: mvnun did not set inform" };
        break;
    };

    return value;
}

