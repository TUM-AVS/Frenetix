#ifndef CARTESIANSAMPLEBINDING_HPP
#define CARTESIANSAMPLEBINDING_HPP

//pybind includes

#include <nanobind/nanobind.h>
#include <nanobind/stl/vector.h>
#include <nanobind/eigen/dense.h>

#include "trajectory/CartesianSample.hpp"

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindCartesianSample(nb::module_ &m);

} //plannerCPP


#endif //CARTESIANSAMPLEBINDING_HPP
