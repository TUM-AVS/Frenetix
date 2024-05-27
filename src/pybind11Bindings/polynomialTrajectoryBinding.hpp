#pragma once

//pybind includes

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "polynomial.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindPolynomialTrajectory(nb::module_ &m);

} //plannerCPP

