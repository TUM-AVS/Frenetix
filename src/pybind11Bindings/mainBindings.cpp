#include <Eigen/Dense>
#include <vector>
#include <memory>

//pybind includes
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/eigen.h>

//Other Bindings
#include "handlerBinding.hpp"
#include "cartesianSampleBinding.hpp"
#include "coordinateSystemWrapperBinding.hpp"
#include "curviLinearSampleBinding.hpp"
#include "polynomialTrajectoryBinding.hpp"
#include "trajectoryStrategyBinding.hpp"
#include "trajectorySampleBinding.hpp"
#include "geometryMsgsBinding.hpp"

namespace py = pybind11;
namespace plannerCPP
{
    PYBIND11_MODULE(_frenetix, m)
    {
        initBindHandler(m);
        initBindCoordinateSystemWrapper(m);
        initBindCartesianSample(m);
        initBindCurviLinearSample(m);
        initBindPolynomialTrajectory(m);
        initBindTrajectoryStrategy(m);
        initBindTrajectorySample(m);
        initBindGeometryMsg(m);
    }
} //plannerCPP



