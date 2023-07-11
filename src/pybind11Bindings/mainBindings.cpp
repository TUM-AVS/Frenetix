#include <Eigen/Dense>
#include <vector>
#include <memory>

//pybind includes
#include <nanobind/ndarray.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/bind_vector.h>
#include <nanobind/eigen/dense.h>

//Other Bindings
#include "handlerBinding.hpp"
#include "cartesianSampleBinding.hpp"
#include "coordinateSystemWrapperBinding.hpp"
#include "curviLinearSampleBinding.hpp"
#include "polynomialTrajectoryBinding.hpp"
#include "trajectoryStrategyBinding.hpp"
#include "trajectorySampleBinding.hpp"
#include "geometryMsgsBinding.hpp"

namespace nb = nanobind;
namespace plannerCPP
{
    NB_MODULE(frenetPlannerHelper, m) 
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



