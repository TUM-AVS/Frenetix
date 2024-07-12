set(FP_SRC_FILES
  "CoordinateSystemWrapper.cpp"
  "TrajectoryHandler.cpp"
  "geometryMsgs.cpp"
  "strategies/CostFunctions/CalculateAccelerationCost.cpp"
  "strategies/CostFunctions/CalculateCollisionProbabilityFast.cpp"
  "strategies/CostFunctions/CalculateCollisionProbabilityMahalanobis.cpp"
  "strategies/CostFunctions/CalculateDistanceToObstacleCost.cpp"
  "strategies/CostFunctions/CalculateDistanceToReferencePathCost.cpp"
  "strategies/CostFunctions/CalculateJerkCost.cpp"
  "strategies/CostFunctions/CalculateLaneCenterOffsetCost.cpp"
  "strategies/CostFunctions/CalculateLateralAccelerationCost.cpp"
  "strategies/CostFunctions/CalculateLateralJerkCost.cpp"
  "strategies/CostFunctions/CalculateLateralVelocityCost.cpp"
  "strategies/CostFunctions/CalculateLongitudinalAccelerationCost.cpp"
  "strategies/CostFunctions/CalculateLongitudinalJerkCost.cpp"
  "strategies/CostFunctions/CalculateLongitudinalVelocityCost.cpp"
  "strategies/CostFunctions/CalculateNegativeAccelerationCost.cpp"
  "strategies/CostFunctions/CalculateNegativeOrientationOffsetCost.cpp"
  "strategies/CostFunctions/CalculateNegativeVelocityOffsetCost.cpp"
  "strategies/CostFunctions/CalculateOrientationOffsetCost.cpp"
  "strategies/CostFunctions/CalculatePositiveAccelerationCost.cpp"
  "strategies/CostFunctions/CalculatePositiveOrientationOffsetCost.cpp"
  "strategies/CostFunctions/CalculatePositiveVelocityOffsetCost.cpp"
  "strategies/CostFunctions/CalculateSteeringAngleCost.cpp"
  "strategies/CostFunctions/CalculateSteeringRateCost.cpp"
  "strategies/CostFunctions/CalculateVelocityOffsetCost.cpp"
  "strategies/CostFunctions/CalculateYawCost.cpp"
  "strategies/FeasabilityFunctions/CheckAccelerationConstraint.cpp"
  "strategies/FeasabilityFunctions/CheckCurvatureConstraints.cpp"
  "strategies/FeasabilityFunctions/CheckCurvatureRateConstrains.cpp"
  "strategies/FeasabilityFunctions/CheckVelocityConstraints.cpp"
  "strategies/FeasabilityFunctions/CheckYawRateConstraint.cpp"
  "strategies/FeasabilityFunctions/CheckYawRateConstraint.cpp"
  "strategies/Functions/ComputeInitalState.cpp"
  "strategies/Functions/FillCoordinates.cpp"
  "trajectory/CartesianSample.cpp"
  "trajectory/CurvilinearSample.cpp"
  "trajectory/TrajectorySample.cpp"
  "util.cpp"
)

set(FP_HDR_FILES
  "CoordinateSystemWrapper.hpp"
  "TrajectoryHandler.hpp"
  "geometryMsgs.hpp"
  "strategies/CostFunctions/CalculateAccelerationCost.hpp"
  "strategies/CostFunctions/CalculateCollisionProbabilityFast.hpp"
  "strategies/CostFunctions/CalculateCollisionProbabilityMahalanobis.hpp"
  "strategies/CostFunctions/CalculateDistanceToObstacleCost.hpp"
  "strategies/CostFunctions/CalculateDistanceToReferencePathCost.hpp"
  "strategies/CostFunctions/CalculateJerkCost.hpp"
  "strategies/CostFunctions/CalculateLaneCenterOffsetCost.hpp"
  "strategies/CostFunctions/CalculateLateralAccelerationCost.hpp"
  "strategies/CostFunctions/CalculateLateralJerkCost.hpp"
  "strategies/CostFunctions/CalculateLateralVelocityCost.hpp"
  "strategies/CostFunctions/CalculateLongitudinalAccelerationCost.hpp"
  "strategies/CostFunctions/CalculateLongitudinalJerkCost.hpp"
  "strategies/CostFunctions/CalculateLongitudinalVelocityCost.hpp"
  "strategies/CostFunctions/CalculateNegativeAccelerationCost.hpp"
  "strategies/CostFunctions/CalculateNegativeOrientationOffsetCost.hpp"
  "strategies/CostFunctions/CalculateNegativeVelocityOffsetCost.hpp"
  "strategies/CostFunctions/CalculateOrientationOffsetCost.hpp"
  "strategies/CostFunctions/CalculatePositiveAccelerationCost.hpp"
  "strategies/CostFunctions/CalculatePositiveOrientationOffsetCost.hpp"
  "strategies/CostFunctions/CalculatePositiveVelocityOffsetCost.hpp"
  "strategies/CostFunctions/CalculateSteeringAngleCost.hpp"
  "strategies/CostFunctions/CalculateSteeringRateCost.hpp"
  "strategies/CostFunctions/CalculateVelocityOffsetCost.hpp"
  "strategies/CostFunctions/CalculateYawCost.hpp"
  "strategies/FeasabilityFunctions/CheckAccelerationConstraint.hpp"
  "strategies/FeasabilityFunctions/CheckCurvatureConstraints.hpp"
  "strategies/FeasabilityFunctions/CheckCurvatureRateConstrains.hpp"
  "strategies/FeasabilityFunctions/CheckVelocityConstraints.hpp"
  "strategies/FeasabilityFunctions/CheckYawRateConstraint.hpp"
  "strategies/FeasabilityFunctions/CheckYawRateConstraint.hpp"
  "strategies/Functions/ComputeInitalState.hpp"
  "strategies/Functions/FillCoordinates.hpp"
  "trajectory/CartesianSample.hpp"
  "trajectory/CurvilinearSample.hpp"
  "trajectory/TrajectorySample.hpp"
  "util.hpp"
)

set(FP_PYTHON_SRC_FILES
  "pybind11Bindings/cartesianSampleBinding.cpp"
  "pybind11Bindings/coordinateSystemWrapperBinding.cpp"
  "pybind11Bindings/costStrategyBinding.cpp"
  "pybind11Bindings/curviLinearSampleBinding.cpp"
  "pybind11Bindings/feasabilityStrategyBinding.cpp"
  "pybind11Bindings/geometryMsgsBinding.cpp"
  "pybind11Bindings/handlerBinding.cpp"
  "pybind11Bindings/loggingSink.cpp"
  "pybind11Bindings/mainBindings.cpp"
  "pybind11Bindings/polynomialTrajectoryBinding.cpp"
  "pybind11Bindings/trajectoryFunctionsBinding/costFunctionsBinding.cpp"
  "pybind11Bindings/trajectoryFunctionsBinding/feasabilityFunctionsBinding.cpp"
  "pybind11Bindings/trajectoryFunctionsBinding/otherFunctionsBinding.cpp"
  "pybind11Bindings/trajectorySampleBinding.cpp"
  "pybind11Bindings/trajectoryStrategyBinding.cpp"
)

set(FP_PYTHON_HDR_FILES
  "pybind11Bindings/cartesianSampleBinding.hpp"
  "pybind11Bindings/coordinateSystemWrapperBinding.hpp"
  "pybind11Bindings/costStrategyBinding.hpp"
  "pybind11Bindings/curviLinearSampleBinding.hpp"
  "pybind11Bindings/feasabilityStrategyBinding.hpp"
  "pybind11Bindings/geometryMsgsBinding.hpp"
  "pybind11Bindings/handlerBinding.hpp"
  "pybind11Bindings/loggingSink.hpp"
  "pybind11Bindings/polynomialTrajectoryBinding.hpp"
  "pybind11Bindings/trajectoryFunctionsBinding/costFunctionsBinding.hpp"
  "pybind11Bindings/trajectoryFunctionsBinding/feasabilityFunctionsBinding.hpp"
  "pybind11Bindings/trajectoryFunctionsBinding/otherFunctionsBinding.hpp"
  "pybind11Bindings/trajectorySampleBinding.hpp"
  "pybind11Bindings/trajectoryStrategyBinding.hpp"
)

add_library(mathutil OBJECT
        math/mvndst.f

        math/mvn.cpp

        math/covariance.hpp
        math/mvn.hpp
        math/mvndst.hpp
)

target_include_directories(mathutil
        PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/math>
)

target_link_libraries(mathutil
        PUBLIC
        Eigen3::Eigen
        )

add_library(fp_core STATIC ${FP_SRC_FILES} ${FP_HDR_FILES})

if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.23.0)
    target_sources(fp_core
            INTERFACE
            FILE_SET fp_core_headers
            TYPE HEADERS
            # BASE_DIRS ../include
            FILES ${FP_HDR_FILES})
endif()

target_link_libraries(fp_core PUBLIC mathutil)

set_property(TARGET fp_core PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(fp_core
        PUBLIC
        Eigen3::Eigen
        crccosy
        Taskflow::Taskflow
        spdlog::spdlog
        )

target_include_directories(fp_core
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/strategies>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/trajectory>
        )

if(ENABLE_EXPERIMENTAL_OPTIMIZATIONS)
  foreach(target fp_core crccosy)
    target_compile_definitions(${target} PRIVATE ENABLE_EXPERIMENTAL_OPTIMIZATIONS)
  endforeach()
endif()

# add pybind module

if(SKBUILD)
    set(python_module_name _frenetix)
    nanobind_add_module(${python_module_name}
        ${FP_PYTHON_SRC_FILES} ${FP_PYTHON_HDR_FILES}
    )

    target_link_libraries(${python_module_name} PRIVATE fp_core)
        
    nanobind_add_stub(
        _frenetix_stub
        MODULE _frenetix
        OUTPUT _frenetix/__init__.pyi
        PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
        DEPENDS ${python_module_name}
        MARKER_FILE py.typed
        INCLUDE_PRIVATE
        VERBOSE
    )

    nanobind_add_stub(
        _frenetix_trajectory_functions_stub
        MODULE _frenetix.trajectory_functions
        OUTPUT _frenetix/trajectory_functions/__init__.pyi
        PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
        DEPENDS ${python_module_name}
        INCLUDE_PRIVATE
        VERBOSE
    )

    nanobind_add_stub(
        _frenetix_cost_functions_stub
        MODULE _frenetix.trajectory_functions.cost_functions
        OUTPUT _frenetix/trajectory_functions/cost_functions/__init__.pyi
        PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
        DEPENDS ${python_module_name}
        INCLUDE_PRIVATE
        VERBOSE
    )

    nanobind_add_stub(
        _frenetix_feasability_functions_stub
        MODULE _frenetix.trajectory_functions.feasability_functions
        OUTPUT _frenetix/trajectory_functions/feasability_functions/__init__.pyi
        PYTHON_PATH $<TARGET_FILE_DIR:${python_module_name}>
        DEPENDS ${python_module_name}
        INCLUDE_PRIVATE
        VERBOSE
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/py.typed
        DESTINATION frenetix
        COMPONENT PythonModules
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/_frenetix/__init__.pyi
        DESTINATION frenetix/_frenetix
        COMPONENT PythonModules
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/_frenetix/trajectory_functions/__init__.pyi
        DESTINATION frenetix/_frenetix/trajectory_functions
        COMPONENT PythonModules
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/_frenetix/trajectory_functions/cost_functions/__init__.pyi
        DESTINATION frenetix/_frenetix/trajectory_functions/cost_functions
        COMPONENT PythonModules
    )

    install(FILES
        ${CMAKE_CURRENT_BINARY_DIR}/_frenetix/trajectory_functions/feasability_functions/__init__.pyi
        DESTINATION frenetix/_frenetix/trajectory_functions/feasability_functions
        COMPONENT PythonModules
    )

    if(FALSE)
        install(DIRECTORY
            ${CMAKE_CURRENT_SOURCE_DIR}/python/frenetix
            DESTINATION ${SKBUILD_PLATLIB_DIR}
            COMPONENT PythonModules
            PATTERN "__pycache__" EXCLUDE
        )
    endif()

    install(TARGETS ${python_module_name}
            LIBRARY DESTINATION frenetix
            COMPONENT PythonModules
            )

    add_custom_target(install-python-modules-${python_module_name}
        ${CMAKE_COMMAND}
        -DCMAKE_INSTALL_COMPONENT=PythonModules
        -P "${PROJECT_BINARY_DIR}/cmake_install.cmake"
        DEPENDS ${python_module_name}
        )
endif(SKBUILD)

