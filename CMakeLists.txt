# NOTE: The upper CMake version bound specified here does not prevent using newer
# CMake versions - rather, it simply tells CMake that we are aware of versions
# in this range, allowing CMake to adapt its behaviour accordingly.
#
# Citing the documentation:
# The cmake_minimum_required(VERSION) command implicitly invokes the cmake_policy(VERSION)
# command to specify that the current project code is written for the given range of CMake versions.
# Source: https://cmake.org/cmake/help/v3.25/command/cmake_minimum_required.html
cmake_minimum_required(VERSION 3.24..3.26)

if(NOT DEFINED SKBUILD_PROJECT_NAME)
    set(SKBUILD_PROJECT_NAME frenetPlannerHelper)
    message(STATUS "SKBUILD_PROJECT_NAME not defined, falling back to ${SKBUILD_PROJECT_NAME}")
endif()
if(NOT DEFINED SKBUILD_PROJECT_VERSION)
    set(SKBUILD_PROJECT_VERSION 0.0.1)
    message(STATUS "SKBUILD_PROJECT_VERSION not defined, falling back to ${SKBUILD_PROJECT_VERSION}")
else()
    # Strip version suffix (CMake only allows integer version components)
    string(REGEX MATCH "^[0-9]+\.[0-9]+\.[0-9]+" SKBUILD_PROJECT_VERSION ${SKBUILD_PROJECT_VERSION})
endif()

project(${SKBUILD_PROJECT_NAME}
    LANGUAGES CXX Fortran
    # TODO: Find an automatic way to sync the version with setuptools_scm/Git tags
    VERSION ${SKBUILD_PROJECT_VERSION}
    )

# CMP0077 (3.13) - option() honors normal variables.
# Relevant for Eigen3
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# CMP0063 - Honor visibility properties for all target types
# TODO: Probably not required anymore - check all configurations and remove if possible
# cmake_policy(SET CMP0063 NEW)

# CMP0126 (3.21) - Removal of normal variables by set(CACHE)
if(POLICY CMP0126)
    cmake_policy(SET CMP0126 NEW)
endif()

# CMP0135 - URL download timestamp
if(POLICY CMP0135)
    cmake_policy(SET CMP0135 NEW)
endif()

# Adapted from Eigen3 - snippet to get a value for PROJECT_IS_TOP_LEVEL
# on CMake versions before v3.21.0
if(CMAKE_VERSION VERSION_LESS 3.21.0)
  if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(PROJECT_IS_TOP_LEVEL ON)
  else()
    set(PROJECT_IS_TOP_LEVEL OFF)
  endif()
  set(${PROJECT_NAME}_IS_TOP_LEVEL ${PROJECT_IS_TOP_LEVEL})
endif()

if(NOT SKBUILD)
    set(CMAKE_VERIFY_INTERFACE_HEADER_SETS ON)
endif()

include(CMakePrintHelpers)

set(CMAKE_COLOR_DIAGNOSTICS ON)

if(CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_CXX_STANDARD_INCLUDE_DIRECTORIES ${CMAKE_CXX_IMPLICIT_INCLUDE_DIRECTORIES})
endif()

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(AppleOpenMP)

include(FetchContent)
FetchContent_Declare(
  pybind11

  URL            https://github.com/pybind/pybind11/archive/refs/tags/v2.10.4.tar.gz
  URL_HASH       SHA256=832e2f309c57da9c1e6d4542dedd34b24e4192ecb4d62f6f4866a737454c9970

  FIND_PACKAGE_ARGS
)

find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)


# IMPORTANT: DO NOT MOVE this section, in particular the calls to find_package(Python)
# and find_package(pybind11), without careful consideration
if(SKBUILD)
        # Scikit-Build does not add your site-packages to the search path
        # automatically, so we need to add it _or_ the pybind11 specific directory
        # here.
        if(FALSE)
        execute_process(
            COMMAND "${PYTHON_EXECUTABLE}" -c
                    "import pybind11; print(pybind11.get_cmake_dir())"
            OUTPUT_VARIABLE _tmp_dir
            OUTPUT_STRIP_TRAILING_WHITESPACE)
        list(APPEND CMAKE_PREFIX_PATH "${_tmp_dir}")
        endif()
        # set(Python_FIND_VIRTUALENV ONLY)

        set(PYBIND11_FINDPYTHON ON)
        find_package(Python 3.7 REQUIRED COMPONENTS Development.Module)

        # Now we can find pybind11
        find_package(pybind11 2.7.0 CONFIG REQUIRED)

        # SKBUILD_SELF_CONTAINED controls whether we try to build all dependencies
        # ourselves. This is used in order to build cross-platform wheels
        # using cibuildwheel.
        # We don't enable this in normal Python builds since it will generally
        # just slow down the build.
        set(SKBUILD_SELF_CONTAINED OFF)

        if(DEFINED ENV{CIBUILDWHEEL})
            set(SKBUILD_SELF_CONTAINED ON)
        endif()

        message(STATUS "PYTHON MODE - assuming we are invoked by pip/setup.py")
        message(STATUS "PYTHON MODE - building static libraries")

        set(FETCHCONTENT_QUIET ON)

        # Globally build static libraries (affects all calls to add_library
        # without an explicit library type)
        set(BUILD_SHARED_LIBS OFF)
        set(CMAKE_POSITION_INDEPENDENT_CODE ON)

        # Globally set visibility preset
        set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
        set(CMAKE_CXX_VISIBILITY_PRESET hidden)
        set(CMAKE_Fortran_VISIBILITY_PRESET hidden)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# macOS 10.14 is required for full C++17 support
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.14")

# Some extra debugging for project developers - safe to disable, but read on why they might be useful:

# By enabling CMAKE_LINK_LIBRARIES_ONLY_TARGETS, CMake will report errors whenever
# a name that does not refer to a target known to CMake is passed to
# target_link_libraries and friends.
# We use targets for all dependencies in this project, so if a non-target name is
# passed it usually indicates a bug (usually a typo or a missing find_package call)
# in the CMake configuration.
# Example:
#   target_link_libraries(example_library PUBLIC spdlog)
# This is incorrect as spdlog is not always a target name provided by the spdlog project - it might
# exist in the project itself when it is included via FetchContent, but not if we use a system-provided
# spdlog version.
# Normally, CMake would go on to add "spdlog" as a literal library to the linker command line (e.g. -lspdlog),
# which is not what we want as it might not correct to the correct spdlog library *and* it does not ensure
# the usage requirements (include directories, definitions, other compiler options) are correctly added
# to the compiler command line for example_library.
#
# But since we have CMAKE_LINK_LIBRARIES_ONLY_TARGETS enabled, CMake will instead print an error like this:
# CMake Error at CMakeLists.txt:123456 (target_link_libraries):
#   Target "example_library" has LINK_LIBRARIES_ONLY_TARGETS enabled, but it
#   links to:
#
#     spdlog
#
#   which is not a target.  Possible reasons include:
#
#     * There is a typo in the target name.
#     * A find_package call is missing for an IMPORTED target.
#     * An ALIAS target is missing.

set(CMAKE_LINK_LIBRARIES_ONLY_TARGETS ON)

set(CMAKE_MESSAGE_CONTEXT_SHOW ON)

# Compile command database is required for Clang-assisted parsing in Doxygen
# TODO: Set this conditionally
set(CMAKE_EXPORT_COMPILE_COMMANDS ON CACHE INTERNAL "")

# Ensure executables are in the top level directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

include(CMakeDependentOption)

option(BUILD_SHARED_LIBS "Build ${PROJECT_NAME} as a shared library" ON)

option(${PROJECT_NAME}_BUILD_SHARED_LIBS "Build using shared libraries" ${BUILD_SHARED_LIBS})

set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE OFF)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24.0)
    set(CMAKE_SUPPORTS_TRY_FIND_PACKAGE ON)
else()
    message(WARNING "Your CMake version (${CMAKE_VERSION}) does not support "
        "the FetchContent find_package integration introduced in CMake 3.24. "
        "As a fallback, we will simply build all dependencies ourselves "
        "irrespective of whether a suitable system version exists. "
        "While this does not impair functionality, it might slow down the build "
        "process a bit.\n"
        "In case you have all required dependencies installed, you can try "
        "enabling the option\n"
        "\t${PROJECT_NAME}_SYSTEM_PACKAGES_FORCE\n"
        "which will force using find_package for all dependencies.")
endif()

option(${PROJECT_NAME}_SYSTEM_PACKAGES "Try to use system packages for dependencies" ON)
cmake_dependent_option(${PROJECT_NAME}_SYSTEM_PACKAGES_FORCE
    "For CMake<3.24: Force using system packages for all dependencies"
    OFF
    "NOT CMAKE_SUPPORTS_TRY_FIND_PACKAGE"
    OFF
    )

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)
FetchContent_Declare(
    commonroad_cmake

    GIT_REPOSITORY https://gitlab.lrz.de/tum-cps/commonroad-cmake.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(commonroad_cmake)

list(APPEND CMAKE_MODULE_PATH ${commonroad_cmake_SOURCE_DIR})

include(toolchain/DiscoverLLD OPTIONAL)
include(toolchain/DiscoverSanitizers OPTIONAL)

# This is a helper script that will automatically add a .gitignore file to the
# binary directory (build directory) so you don't have to do add every build folder
# to your .gitignore.
include(extras/GitIgnoreBinaryDir OPTIONAL)

if(DEFINED ENV{CIBUILDWHEEL} AND CMAKE_SYSTEM_PROCESSOR MATCHES "i686")
    # Ugly hack for broken pthread detection on manylinux2014_i686
    find_library(OpenMP_pthread_LIBRARY NAMES "pthread")
endif()

# Required for proper pthread discovery on some systems
set(THREADS_PREFER_PTHREAD_FLAG TRUE)

find_package(Threads REQUIRED)
find_package(OpenMP REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/external)

include(ExternalBoost)
include(ExternalEigen)
include(ExternalDrivabilityChecker)
include(ExternalTaskflow)

add_subdirectory(src)

