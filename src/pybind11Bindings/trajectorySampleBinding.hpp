#pragma once

//pybind includes
#include <nanobind/nanobind.h>

namespace nb = nanobind;

namespace plannerCPP
{

    void initBindTrajectorySample(nb::module_ &m);

} //plannerCPP

