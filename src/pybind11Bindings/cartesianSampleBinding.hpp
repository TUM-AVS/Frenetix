#ifndef CARTESIANSAMPLEBINDING_HPP
#define CARTESIANSAMPLEBINDING_HPP

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

#include "trajectory/CartesianSample.hpp"

namespace py = pybind11;

namespace plannerCPP
{

    void initBindCartesianSample(pybind11::module &m);

} //plannerCPP


#endif //CARTESIANSAMPLEBINDING_HPP
