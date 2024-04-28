# TODO: List files explicitly
file(GLOB_RECURSE FP_SRC_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS "*.cpp")
file(GLOB_RECURSE FP_HDR_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} CONFIGURE_DEPENDS "*.hpp")

list(FILTER FP_SRC_FILES EXCLUDE REGEX "math/")
list(FILTER FP_HDR_FILES EXCLUDE REGEX "math/")

set(FP_PYTHON_SRC_FILES ${FP_SRC_FILES})
set(FP_PYTHON_HDR_FILES ${FP_HDR_FILES})

list(FILTER FP_SRC_FILES EXCLUDE REGEX "pybind11Bindings/")
list(FILTER FP_HDR_FILES EXCLUDE REGEX "pybind11Bindings/")
list(FILTER FP_PYTHON_SRC_FILES INCLUDE REGEX "pybind11Bindings/")
list(FILTER FP_PYTHON_HDR_FILES INCLUDE REGEX "pybind11Bindings/")

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
        Taskflow
        )

target_include_directories(fp_core
        PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/strategies>
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/src/trajectory>
        )

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

