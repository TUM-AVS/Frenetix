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
    message(STATUS "The project is built using scikit-build")
endif()
if(pybind11_FOUND AND NOT SKBUILD)
    message(STATUS "Adding Python interface for compilation only")
endif()
if(pybind11_FOUND)
        add_library(fp_python OBJECT ${FP_PYTHON_SRC_FILES} ${FP_PYTHON_HDR_FILES})
        target_link_libraries(fp_python PRIVATE fp_core pybind11::module)
        set_property(TARGET fp_python PROPERTY POSITION_INDEPENDENT_CODE ON)
endif()

if(SKBUILD)
    set(python_module_name _frenetix)
    pybind11_add_module(${python_module_name} MODULE $<TARGET_OBJECTS:fp_python>)

    target_link_libraries(${python_module_name} PRIVATE fp_core)

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

