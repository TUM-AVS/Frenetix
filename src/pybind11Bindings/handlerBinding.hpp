#pragma once

//pybind includes
#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindHandler(nb::module_ &m);

} //plannerCPP

