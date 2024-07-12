#include <math/covariance.hpp>
#include <nanobind/nanobind.h>
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

#include "loggingSink.hpp"

namespace nb = nanobind;
namespace plannerCPP
{
    NB_MODULE(_frenetix, m)
    {
        // Disable leak warnings in Release builds
        #ifdef NDEBUG
        nb::set_leak_warnings(false);
        #endif

        // Unconditionally setup default logger
        setup_logger();

        m.def("setup_logger",
            [] (const nb::object& logger) { setup_logger(logger); },
            nb::arg("logger")
        );

        // NOTE: Order of these bindings is critical to ensure proper stubs generation
        initBindCoordinateSystemWrapper(m);

        initBindCartesianSample(m);
        initBindCurviLinearSample(m);

        initBindPolynomialTrajectory(m);
        initBindTrajectorySample(m);
        initBindGeometryMsg(m);

        initBindTrajectoryStrategy(m);

        initBindHandler(m);

        nb::exception<invalid_covariance_matrix_error>(m, "InvalidCovarianceMatrixError", PyExc_ValueError);
    }
} //plannerCPP



