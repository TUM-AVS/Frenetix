#pragma once

#include <cstdint>

extern "C" double bvnmvn_(
    const double *lower,
    const double *upper,
    const int32_t *infin,
    const double *correl
);

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

