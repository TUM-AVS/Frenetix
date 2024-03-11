#include <math/covariance.hpp>
#include <pybind11/pybind11.h>
#include <pyerrors.h>

#include "cartesianSampleBinding.hpp"
#include "coordinateSystemWrapperBinding.hpp"
#include "curviLinearSampleBinding.hpp"
#include "geometryMsgsBinding.hpp"
//Other Bindings
#include "handlerBinding.hpp"
#include "polynomialTrajectoryBinding.hpp"
#include "trajectorySampleBinding.hpp"
#include "trajectoryStrategyBinding.hpp"

namespace py = pybind11;
namespace plannerCPP
{
    PYBIND11_MODULE(_frenetix, m)
    {
        // NOTE: Order of these bindings is critical to ensure proper stubs generation
        initBindCoordinateSystemWrapper(m);

        initBindCartesianSample(m);
        initBindCurviLinearSample(m);

        initBindPolynomialTrajectory(m);
        initBindTrajectorySample(m);
        initBindGeometryMsg(m);

        initBindTrajectoryStrategy(m);

        initBindHandler(m);

        py::register_exception<invalid_covariance_matrix_error>(m, "InvalidCovarianceMatrixError", PyExc_ValueError);
    }
} //plannerCPP



